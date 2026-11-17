"""The online lookup flow: cache first, then Nominatim, never hanging.

Geocoder.lookup(query) decides, for one destination name:

1. A fresh cached answer -> use it (no request).
2. Online search switched off -> a cached answer of any age, else DISABLED.
3. The rate limiter says no -> a cached answer of any age, else RATE_LIMITED.
4. Otherwise wait if needed (>= 1.1 s between requests), send ONE request
   (never retried), parse it, save it in the cache (also "no results").
5. The request failed -> a cached answer of any age (marked stale), else
   OFFLINE / RATE_LIMITED / ERROR.

The HTTP request itself is a function passed in (fetch(params) -> parsed
JSON, raising FetchError), so this module never touches the network and
tests use fakes. Clocks and sleep are passed in for the same reason.

Usage policy of the public server:
https://operations.osmfoundation.org/policies/nominatim/
"""

from dataclasses import dataclass
from enum import IntEnum
import time
from typing import Tuple

from guide_core.geocoding import (
    GeocodeConfig, GeocodeError, parse_results, search_params, searchable_query)
from guide_core.rate_limit import RateLimiter


class Status(IntEnum):
    """Outcome of a lookup; values match guide_interfaces/srv/Geocode.srv."""

    OK = 0             # places found (online or from the cache)
    NO_RESULTS = 1     # searched (now or earlier): nothing in our area
    DISABLED = 2       # online search off and nothing cached
    OFFLINE = 3        # network error, timeout or server error, nothing cached
    RATE_LIMITED = 4   # hourly cap or server refusal, nothing cached
    ERROR = 5          # bad query or an answer we cannot read, nothing cached


class FetchError(Exception):
    """The request failed. kind: 'offline', 'refused' (HTTP 403/429) or 'bad_answer'."""

    KINDS = ('offline', 'refused', 'bad_answer')

    def __init__(self, message, kind='offline'):
        if kind not in self.KINDS:
            raise ValueError(f'kind must be one of {self.KINDS}, got {kind!r}')
        super().__init__(message)
        self.kind = kind


_FAILURE_STATUS = {'offline': Status.OFFLINE, 'refused': Status.RATE_LIMITED,
                   'bad_answer': Status.ERROR}


@dataclass(frozen=True)
class LookupResult:
    """Answer of Geocoder.lookup().

    places: in Nominatim's order; () unless status is OK.
    from_cache: the places come from the cache, not from a request now.
    stale: from the cache and older than its maximum age (a refresh was not
        possible or failed).
    message: what happened, for logs (not for speech).
    """

    status: Status
    query: str
    places: Tuple = ()
    from_cache: bool = False
    stale: bool = False
    message: str = ''


def _age_text(seconds):
    if seconds < 3600.0:
        return f'{seconds / 60.0:.0f} min'
    if seconds < 86400.0:
        return f'{seconds / 3600.0:.0f} h'
    return f'{seconds / 86400.0:.0f} days'


class Geocoder:
    """One lookup at a time (the node is single-threaded)."""

    def __init__(self, bbox, cache, fetch, config=None, limiter=None, online=True,
                 wall_clock=time.time, monotonic=time.monotonic, sleep=time.sleep):
        """Set up the flow.

        bbox: the graph's area (geocoding.BBox). cache: a loaded GeocodeCache.
        fetch: function(params dict) -> parsed JSON, raising FetchError.
        online: whether requests may be sent (changeable later: .online).
        wall_clock: Unix time for the cache; monotonic: for the rate limiter.
        """
        self.bbox = bbox
        self.cache = cache
        self.config = config or GeocodeConfig()
        self.limiter = limiter or RateLimiter()
        self.online = online
        self._fetch = fetch
        self._wall_clock = wall_clock
        self._monotonic = monotonic
        self._sleep = sleep
        self._area = bbox.shrunk(self.config.edge_margin)

    def _from_cache(self, query, hit, note):
        status = Status.OK if hit.places else Status.NO_RESULTS
        found = f'{len(hit.places)} place(s)' if hit.places else 'no results'
        return LookupResult(status, query, hit.places, True, not hit.fresh,
                            f'From cache (saved {_age_text(hit.age)} ago): {found}{note}')

    def lookup(self, query):
        """Find places for one destination name (see the module docstring)."""
        text = searchable_query(query, self.config)
        if not text:
            return LookupResult(Status.ERROR, ' '.join(str(query).split()),
                                message='Query too short to search for')
        hit = self.cache.get(text, self._wall_clock(), self._area)
        if hit is not None and hit.fresh:
            return self._from_cache(text, hit, '')

        if not self.online:
            if hit is not None:
                return self._from_cache(text, hit, ' (stale; online search is off)')
            return LookupResult(Status.DISABLED, text,
                                message='Online search is switched off and the query '
                                        'is not in the cache')

        decision = self.limiter.check(self._monotonic())
        if not decision.send:
            if hit is not None:
                return self._from_cache(text, hit, f' (stale; {decision.reason})')
            return LookupResult(Status.RATE_LIMITED, text, message=decision.reason)
        if decision.wait > 0.0:
            self._sleep(decision.wait)

        params = search_params(text, self.bbox, self.config)
        started = self._monotonic()
        self.limiter.record_request(started)
        try:
            data = self._fetch(params)
            parsed = parse_results(data, self.bbox, self.config)
        except (FetchError, GeocodeError) as e:
            kind = e.kind if isinstance(e, FetchError) else 'bad_answer'
            if kind == 'refused':
                self.limiter.record_refused(self._monotonic())
            if hit is not None:
                return self._from_cache(text, hit, f' (stale; refresh failed: {e})')
            return LookupResult(_FAILURE_STATUS[kind], text, message=f'Search failed: {e}')

        elapsed_ms = (self._monotonic() - started) * 1000.0
        note = ''
        try:
            if not self.cache.put(text, parsed.places, self._wall_clock()):
                note = '; cache is read-only, not saved'
        except OSError as e:
            note = f'; could not save the cache: {e}'
        dropped = []
        if parsed.outside:
            dropped.append(f'{parsed.outside} outside the area')
        if parsed.duplicates:
            dropped.append(f'{parsed.duplicates} duplicate(s)')
        if parsed.invalid:
            dropped.append(f'{parsed.invalid} unreadable')
        found = f'{len(parsed.places)} place(s)' if parsed.places else 'no results'
        message = (f'Nominatim: {found} in {elapsed_ms:.0f} ms'
                   + (f' ({", ".join(dropped)} dropped)' if dropped else '') + note)
        status = Status.OK if parsed.places else Status.NO_RESULTS
        return LookupResult(status, text, parsed.places, False, False, message)

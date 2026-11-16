"""The language model interpreter (qwen_interpreter): it matches a need ("I'm hungry") to one stored place or object.

It only ever answers with a name from the lists and a confidence. It cannot start follow, manual, stop or help: the rules
decide those, so they never wait for it. Claude is asked first; the local model answers when Claude is not available.
"""
import contextlib
import io
import json
import threading
import unittest
import urllib.error
from unittest import mock

import middleman
from middleman import Middleman, Mode, classify, qwen_interpreter

PLACES = ['bathroom', 'kitchen', 'exit', 'pret', 'balcony']


@contextlib.contextmanager
def models(claude=None, ollama=None):
    """Patch the two models. claude is the text llm.ask_cloud returns (None: Claude is not available); ollama is what
    urlopen does (a function, or an exception to raise). Yields the Claude and Ollama mocks."""
    ollama = ollama or mock.Mock(side_effect=AssertionError('Ollama was asked'))
    with mock.patch.object(middleman.llm, 'ask_cloud', return_value=claude) as ask_cloud, \
            mock.patch.object(middleman.urllib.request, 'urlopen', ollama) as urlopen:
        yield ask_cloud, urlopen


def ollama_says(match, confidence=0.9):
    body = json.dumps({'message': {'content': json.dumps({'match': match, 'confidence': confidence})}}).encode()
    return mock.Mock(side_effect=lambda *a, **k: io.BytesIO(body))


def claude_says(match, confidence=0.95):
    return json.dumps({'match': match, 'confidence': confidence})


def understand(match, text='pete get me a coffee', confidence=0.9):
    """What classify() makes of the request when the local model (Claude is not available) answers with match."""
    with models(ollama=ollama_says(match, confidence)):
        return classify(text, PLACES, interpreter=qwen_interpreter(PLACES))


class InterpreterTest(unittest.TestCase):
    def check(self, decision, mode, target=''):
        self.assertEqual((decision.mode, decision.target), (mode, target))

    def test_a_need_is_matched_to_a_stored_place_or_object(self):
        self.check(understand('pret', 'pete get me a coffee'), Mode.LOCATION, 'pret')
        self.check(understand('bathroom', 'pete i have to wash my hands'), Mode.LOCATION, 'bathroom')
        self.check(understand('cup', 'pete get me something to drink from'), Mode.OBJECT, 'cup')
        self.assertEqual(understand('pret').reply, 'Going to the pret.')

    def test_not_sure_or_no_match_leaves_the_rules_answer(self):
        self.assertEqual(understand('pret', confidence=0.69).reply, 'Say that again.')  # under the threshold
        self.assertEqual(understand('', confidence=0.95).reply, 'Say that again.')  # "none of these"
        self.assertEqual(understand('pret', confidence='very').reply, 'Say that again.')  # a confidence that is not a number

    def test_a_match_that_is_not_stored_is_ignored(self):
        self.assertEqual(understand('unicorn', confidence=0.99).reply, 'Say that again.')

    def test_it_cannot_start_a_mode_other_than_a_place_or_object(self):
        for answer in ('follow', 'manual', 'stop', 'help', 'gps'):
            with self.subTest(answer):
                self.assertEqual(understand(answer, confidence=0.99).reply, 'Say that again.')

    def test_rules_decide_when_the_model_is_unreachable_or_unreadable(self):
        with models(ollama=mock.Mock(side_effect=urllib.error.URLError('no ollama'))):
            interpreter = qwen_interpreter(PLACES)
            self.check(classify('pete take me to the bathroom', PLACES, interpreter=interpreter), Mode.LOCATION, 'bathroom')
            self.assertEqual(classify('pete get me a coffee', PLACES, interpreter=interpreter).reply, 'Say that again.')
        with models(ollama=mock.Mock(side_effect=lambda *a, **k: io.BytesIO(b'{"message": {"content": "not json"}}'))):
            self.assertEqual(classify('pete get me a coffee', PLACES, interpreter=qwen_interpreter(PLACES)).reply, 'Say that again.')

    def test_commands_and_requests_the_rules_understand_never_reach_the_model(self):
        with models(claude=claude_says('pret', 0.99)) as (ask_cloud, _):
            interpreter = qwen_interpreter(PLACES)
            for text in ('pete stop', "pete i'm lost", 'pete follow alice', 'pete manual mode', 'pete take me to the bathroom',
                         'pete where is the cup', "pete what's in front of me"):
                with self.subTest(text):
                    classify(text, PLACES, interpreter=interpreter)
            ask_cloud.assert_not_called()

    def test_claude_is_asked_first_and_the_local_model_is_the_fallback(self):
        with models(claude=claude_says('pret')):  # Ollama is not allowed to be asked here
            self.check(classify('pete get me a coffee', PLACES, interpreter=qwen_interpreter(PLACES)), Mode.LOCATION, 'pret')
        with models(ollama=ollama_says('kitchen')) as (ask_cloud, urlopen):  # Claude gives None: no key, offline or slow
            self.check(classify('pete get me a coffee', PLACES, interpreter=qwen_interpreter(PLACES)), Mode.LOCATION, 'kitchen')
            ask_cloud.assert_called()
            urlopen.assert_called()

    def test_an_unknown_named_place_needs_claude_to_be_very_sure(self):
        text = "pete walk me to the dean's office"  # a named place that is not stored: the local model always guesses wrong
        self.assertIn("don't know where", classify(text, PLACES).reply)
        with models(ollama=ollama_says('exit', 0.95)):  # the local model never overrides the refusal
            self.assertIn("don't know where", classify(text, PLACES, interpreter=qwen_interpreter(PLACES)).reply)
        with models(claude=claude_says('exit', 0.8)):  # Claude, but not sure enough
            self.assertIn("don't know where", classify(text, PLACES, interpreter=qwen_interpreter(PLACES)).reply)
        with models(claude=claude_says('exit', 0.95)):
            self.check(classify(text, PLACES, interpreter=qwen_interpreter(PLACES)), Mode.LOCATION, 'exit')

    def test_the_model_is_told_the_places_with_what_they_are_for(self):
        with models(ollama=ollama_says('', 0)) as (_, urlopen):
            classify('pete get me a coffee', ['pret', 'kitchen'], interpreter=qwen_interpreter(['pret', 'kitchen']))
        body = json.loads(urlopen.call_args.args[0].data)
        self.assertIn('pret (coffee, sandwiches, quick food)', body['messages'][0]['content'])  # a place with a purpose line
        self.assertEqual(body['format']['properties']['match']['enum'][:2], ['pret', 'kitchen'])


class MiddlemanWithInterpreterTest(unittest.TestCase):
    def test_a_stop_is_handled_while_the_model_is_slow_and_cancels_the_slow_request(self):
        release = threading.Event()
        asked = threading.Event()

        def slow(text, minimum=0.7):
            asked.set()
            release.wait(5)
            return middleman.resolve_known('kitchen', text, PLACES, {})

        bot = Middleman(PLACES, interpreter=slow)
        results = []
        thread = threading.Thread(target=lambda: results.append(bot.handle('pete get me a coffee')))
        thread.start()
        self.assertTrue(asked.wait(2))
        stop = bot.handle('pete stop')  # while the first request is still with the model
        self.assertTrue(stop.stop)
        self.assertEqual(bot.mode, Mode.IDLE)
        release.set()
        thread.join(5)
        self.assertEqual(results, [None])  # dropped: it cannot start a mode after the stop
        self.assertEqual(bot.mode, Mode.IDLE)

    def test_a_slow_request_that_finishes_without_a_stop_still_applies(self):
        bot = Middleman(PLACES, interpreter=lambda text, minimum=0.7: middleman.resolve_known('kitchen', text, PLACES, {}))
        decision = bot.handle('pete get me a coffee')
        self.assertEqual((decision.mode, decision.target), (Mode.LOCATION, 'kitchen'))
        self.assertEqual(bot.mode, Mode.LOCATION)

    def test_the_model_does_not_slow_a_stop_or_help(self):
        bot = Middleman(PLACES, interpreter=mock.Mock(side_effect=AssertionError('the model was asked')))
        self.assertTrue(bot.handle('pete stop').stop)
        self.assertEqual(bot.handle("pete i'm lost").mode, Mode.HELP)


if __name__ == '__main__':
    unittest.main()

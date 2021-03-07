const express = require('express')
const Say = require('say').Say
const say = new Say('darwin')

const app = express();

let isSpeaking = false;

function speak(text) {
    if (isSpeaking) {
        console.log('error: already speaking');
        return;
    }

    isSpeaking = true;

    console.log('start speech:' + text);

    say.speak(text, 'Alex', 1, () => {
        console.log('finish speech:' + text);
        isSpeaking = false;
    });
}

app.get('/say', function (req, res) {
    console.log('try to speak: ' + req.query.text)
    speak(req.query.text);
    return res.send('ok');
})

const host = "0.0.0.0";
const port = 8080;

app.listen(port, host)
console.log('started')

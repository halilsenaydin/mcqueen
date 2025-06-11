const micButton = document.getElementById('micButton');
const voiceDisplay = document.getElementById('voiceDisplay');
const pulse = document.getElementById('pulse');
const planetAura = document.getElementById('planetAura');
const listening = true;
let recognition;
let audioContext, analyser, dataArray, source;

startMic();

async function startMic() {
  const stream = await navigator.mediaDevices.getUserMedia({
    audio: true,
  });

  audioContext = new (window.AudioContext || window.webkitAudioContext)();
  analyser = audioContext.createAnalyser();
  source = audioContext.createMediaStreamSource(stream);
  source.connect(analyser);
  analyser.fftSize = 256;
  dataArray = new Uint8Array(analyser.frequencyBinCount);

  recognition = new (window.SpeechRecognition ||
    window.webkitSpeechRecognition)();
  recognition.continuous = true;
  recognition.interimResults = false;
  recognition.lang = 'tr-TR';

  recognition.onresult = (event) => {
    const transcript =
      event.results[event.results.length - 1][0].transcript.trim();

    // AJAX POST request
    fetch('/voice-assistant/command', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ command: transcript }),
    })
      .then((response) => response.json())
      .then((data) => {
        // Voice
        voiceDisplay.textContent = data.message;
      })
      .catch(() => {
        // Error
      });
  };

  recognition.onend = () => {
    if (listening) recognition.start();
  };

  recognition.start();

  animate();
}

function animate() {
  if (!listening) return;

  requestAnimationFrame(animate);

  analyser.getByteFrequencyData(dataArray);

  let volume = dataArray.reduce((a, b) => a + b, 0) / dataArray.length;
  let scale = 1 + volume / 300;

  micButton.style.transform = `translate(0, 0) scale(${scale})`;
  pulse.style.transform = `translate(-50%, -50%) scale(${scale})`;
  pulse.style.opacity = Math.min(0.9, volume / 60);
  planetAura.style.transform = `translateY(-10px) scale(${1 + volume / 200})`;
}

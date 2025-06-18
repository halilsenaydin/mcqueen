const toggle = document.getElementById('buzzer-toggle');
const statusText = document.getElementById('status-text');

function updateStatus(isOn) {
  if (isOn) {
    statusText.textContent = 'Buzzer On';
    statusText.classList.remove('off');
  } else {
    statusText.textContent = 'Buzzer Off';
    statusText.classList.add('off');
  }
}

toggle.addEventListener('change', () => {
  const isOn = toggle.checked;
  updateStatus(isOn);

  // AJAX POST request
  fetch(isOn ? '/buzzer/on' : '/buzzer/off', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
  })
    .then((response) => {
      if (!response.ok) {
        alert('Buzzer command could not be sent!');

        toggle.checked = !isOn;
        updateStatus(!isOn);
      }
    })
    .catch(() => {
      alert('The server is unreachable!');

      toggle.checked = !isOn;
      updateStatus(!isOn);
    });
});

// Update status when page loads
updateStatus(toggle.checked);

const toggle = document.getElementById('headlight-toggle');
const statusText = document.getElementById('status-text');

function updateStatus(isOn) {
  if (isOn) {
    statusText.textContent = 'Headlight On';
    statusText.classList.remove('off');
  } else {
    statusText.textContent = 'Headlight Off';
    statusText.classList.add('off');
  }
}

toggle.addEventListener('change', () => {
  const isOn = toggle.checked;
  updateStatus(isOn);

  // AJAX POST request
  fetch(isOn ? '/headlight/on' : '/headlight/off', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
  })
    .then((response) => {
      if (!response.ok) {
        alert('Headlight command could not be sent!');

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

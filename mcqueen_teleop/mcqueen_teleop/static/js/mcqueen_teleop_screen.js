const picsMap = {
  0: 'opened.png',
  1: 'blink.png',
  2: 'opened-look-right.png',
  3: 'opened-look-left.png',
  4: 'blink-look-right.png',
  5: 'blink-look-left.png',
};
const cards = document.querySelectorAll('.screen-card');
const responseMsg = document.getElementById('response-msg');

cards.forEach((card) => {
  card.addEventListener('click', () => {
    // Highlight the selected card
    cards.forEach((c) => c.classList.remove('selected'));
    card.classList.add('selected');

    const picId = card.getAttribute('data-id');

    // AJAX POST request
    fetch('/screen/change-picture', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ pic_id: picId }),
    })
      .then((response) => response.json())
      .then((data) => {
        responseMsg.style.display = 'block';

        if (data.success) {
          responseMsg.style.color = '#ffb74d';
          responseMsg.textContent = `Image ${picsMap[picId]} was selected and service was called.`;
        } else {
          responseMsg.style.color = 'rgba(242, 246, 107, 0.6);';
          responseMsg.textContent = `Error: ${
            data.error || 'Service call failed.'
          }`;
        }
      })
      .catch(() => {
        responseMsg.style.color = 'rgba(242, 246, 107, 0.6);';
        responseMsg.style.display = 'block';
        responseMsg.textContent =
          'An error occurred while connecting to the server.';
      });
  });
});

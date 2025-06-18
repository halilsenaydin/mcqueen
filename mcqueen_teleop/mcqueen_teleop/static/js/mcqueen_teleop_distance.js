async function fetchDistanceData() {
  try {
    const response = await fetch('/distances');
    if (!response.ok) throw new Error('Data could not be retrieved!');
    const data = await response.json();

    const tbody = document.querySelector('#distance-table tbody');
    tbody.innerHTML = '';

    if (data.length === 0) {
      tbody.innerHTML =
        '<tr><td colspan="3" style="color:#ffb74d; font-style: italic;">No data yet</td></tr>';
      return;
    }

    for (const item of data) {
      const tr = document.createElement('tr');
      tr.innerHTML = `
              <td>${item.time}</td>
              <td>${item.distance.toFixed(2)}</td>
              <td>${item.description || ''}</td>
            `;
      tbody.appendChild(tr);
    }
  } catch (err) {
    console.error('Error:', err);
  }
}

// Refresh data every 2 seconds
setInterval(fetchDistanceData, 2000);
fetchDistanceData();

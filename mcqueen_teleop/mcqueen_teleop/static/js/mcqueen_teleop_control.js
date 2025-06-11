// Steering Wheel Control
const wheelContainer = document.getElementById('steering-wheel-container');
let dragging = false;
let centerX = 0;
let rotation = 0;

wheelContainer.addEventListener('pointerdown', (e) => {
  dragging = true;
  const rect = wheelContainer.getBoundingClientRect();
  centerX = rect.left + rect.width / 2;

  updateRotation(e.clientX);
  wheelContainer.setPointerCapture(e.pointerId); // Çoklu dokunmayı destekler
  e.preventDefault();
});

wheelContainer.addEventListener('pointermove', (e) => {
  if (!dragging) return;
  updateRotation(e.clientX);
});

wheelContainer.addEventListener('pointerup', (e) => {
  dragging = false;
  wheelContainer.releasePointerCapture(e.pointerId);
  wheelContainer.style.filter = 'drop-shadow(0 5px 6px rgba(0,0,0,0.7))';
  updateAria();
});

// Gas Pedal Control
const gasPedal = document.getElementById('gas-pedal');

gasPedal.addEventListener('pointerdown', (e) => {
  e.preventDefault();
  handlePressStart();
});

gasPedal.addEventListener('pointerup', (e) => {
  e.preventDefault();
  handlePressEnd();
});

const gearbox = document.getElementById('gearbox');
const handle = document.getElementById('gear-handle');

// Gearbox Control
const gearHandle = document.getElementById('gear-handle');
const gearPositions = { D: 0, R: 1 };
let gearPosition = 0;

gearHandle.addEventListener('click', () => {
  gearPosition = 1 - gearPosition; // Toggle D-R
  updateGearHandle();
  sendGearState();
});

updateGearHandle();

/**
 * @description Clamps a number between a minimum and maximum value.
 * @param {number} value - The input number to clamp.
 * @param {number} min - The minimum allowed value.
 * @param {number} max - The maximum allowed value.
 * @returns {number} The clamped value within the range [min, max].
 */
function clamp(value, min, max) {
  return Math.min(Math.max(value, min), max);
}

/**
 * @description Updates the ARIA attribute 'aria-valuenow' of the wheel container based on the current rotation value.
 */
function updateAria() {
  wheelContainer.setAttribute('aria-valuenow', rotation.toFixed(0));
}

/**
 * @description Sends the steering angle to the server and updates the eye direction.
 * @param {number} angle - The steering angle to send. Positive values steer right, negative values steer left.
 */
function sendSteeringAngle(angle) {
  console.log(`Steering angle: ${angle}`);
  fetch('/control/steer', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ angle: angle }),
  });

  if (angle > 10) {
    addLookRight();
    removeLookLeft();
  } else if (angle < -10) {
    addLookLeft();
    removeLookRight();
  } else {
    removeLookLeft();
    removeLookRight();
  }
}

/**
 * @description Sends an accelerate command to the server.
 */
function sendAccelerate() {
  fetch('/control/accelerate', {
    method: 'POST',
    headers: {
      'Content-Type': 'application/json',
    },
  });
}

/**
 * @description Sends a brake command to the server.
 */
function sendBrake() {
  fetch('/control/brake', {
    method: 'POST',
    headers: {
      'Content-Type': 'application/json',
    },
  });
}

/**
 * @description Handles the press start event: starts the blinking animation and sends the accelerate command.
 */
function handlePressStart() {
  addBlink();
  sendAccelerate();
}

/**
 * @description Handles the press end event: stops the blinking animation and sends the brake command.
 */
function handlePressEnd() {
  removeBlink();
  sendBrake();
}

/**
 * @description Adds the 'blink' class to the McQueen eyes element to trigger a blinking animation.
 */
function addBlink() {
  const mcqueenEyes = document.getElementById('mcqueen-eyes');

  if (mcqueenEyes.classList.contains('blink')) {
    return;
  }

  mcqueenEyes.classList.add('blink');
}

/**
 * @description Removes the 'blink' class from the McQueen eyes element to stop the blinking animation.
 */
function removeBlink() {
  const mcqueenEyes = document.getElementById('mcqueen-eyes');

  if (!mcqueenEyes.classList.contains('blink')) {
    return;
  }

  mcqueenEyes.classList.remove('blink');
}

/**
 * @description Adds the 'look-left' class to the McQueen eyes element to make the eyes look left.
 */
function addLookLeft() {
  const mcqueenEyes = document.getElementById('mcqueen-eyes');

  if (mcqueenEyes.classList.contains('look-left')) {
    return;
  }

  mcqueenEyes.classList.add('look-left');
}

/**
 * @description Removes the 'look-left' class from the McQueen eyes element to reset eye direction.
 */
function removeLookLeft() {
  const mcqueenEyes = document.getElementById('mcqueen-eyes');

  if (!mcqueenEyes.classList.contains('look-left')) {
    return;
  }

  mcqueenEyes.classList.remove('look-left');
}

/**
 * @description Adds the 'look-right' class to the McQueen eyes element to make the eyes look right.
 */
function addLookRight() {
  const mcqueenEyes = document.getElementById('mcqueen-eyes');

  if (mcqueenEyes.classList.contains('look-right')) {
    return;
  }

  mcqueenEyes.classList.add('look-right');
}

/**
 * @description Removes the 'look-right' class from the McQueen eyes element to reset eye direction.
 */
function removeLookRight() {
  const mcqueenEyes = document.getElementById('mcqueen-eyes');

  if (!mcqueenEyes.classList.contains('look-right')) {
    return;
  }

  mcqueenEyes.classList.remove('look-right');
}

/**
 * @function updateRotation
 * @description Updates the rotation of the wheel based on the horizontal mouse position.
 * @param {number} clientX - The horizontal mouse coordinate (e.g., from a mouse event).
 */
function updateRotation(clientX) {
  // Horizontal difference of center with mouse
  const diffX = clientX - centerX;
  let newRotation = diffX;

  // Limit between -90 and 90
  newRotation = clamp(newRotation, -90, 90);

  rotation = newRotation;
  wheelContainer.style.transform = `rotate(${rotation}deg)`;

  updateAria();
  sendSteeringAngle(rotation);
}

/**
 * @description Updates the position and label of the gear handle element depending on the current gear position (0 for Drive, otherwise Reverse).
 */
function updateGearHandle() {
  let dTop = '10px';
  let rTop = '90px';

  if (window.screen.width > 700) {
    dTop = '10px';
    rTop = '140px';
  }

  if (gearPosition === 0) {
    gearHandle.style.top = dTop;
    gearHandle.textContent = 'D';
  } else {
    gearHandle.style.top = rTop;
    gearHandle.textContent = 'R';
  }
}

/**
 * @description Sends the current gear position to the server via a POST request.
 */
function sendGearState() {
  fetch('/control/gear', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ gear: gearPosition }),
  });
}

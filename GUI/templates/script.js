document.addEventListener('DOMContentLoaded', () => {
    const logArea = document.getElementById('log');
    const speedSlider = document.getElementById('speed');
  
    function log(message) {
      logArea.value += message + '\n';
      logArea.scrollTop = logArea.scrollHeight;
    }
  
    function sendCommand(command) {
      // Replace with actual fetch call to backend
      log(`Sent command: ${command}`);
    }
  
    function readSensor(sensor) {
      // Replace with actual fetch call to backend
      log(`Reading ${sensor} sensor...`);
      // Simulate sensor data
      setTimeout(() => {
        log(`${sensor} sensor value: ${Math.floor(Math.random() * 100)}`);
      }, 500);
    }
  
    document.querySelectorAll('.control-btn').forEach(button => {
      button.addEventListener('mousedown', () => {
        const command = button.getAttribute('data-command');
        sendCommand(command);
      });
      button.addEventListener('mouseup', () => {
        sendCommand('Stop');
      });
    });
  
    document.querySelectorAll('.mode-btn').forEach(button => {
      button.addEventListener('click', () => {
        const command = button.getAttribute('data-command');
        sendCommand(command);
      });
    });
  
    document.querySelectorAll('.sensor-btn').forEach(button => {
      button.addEventListener('click', () => {
        const sensor = button.getAttribute('data-sensor');
        readSensor(sensor);
      });
    });
  
    speedSlider.addEventListener('change', () => {
      const speed = speedSlider.value;
      sendCommand(`V${speed}`);
      log(`Speed set to: ${speed}`);
    });
  });
  
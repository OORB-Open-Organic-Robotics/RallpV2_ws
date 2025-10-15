document.addEventListener('DOMContentLoaded', () => {
  // Navigation UI elements
  const navControlBtn = document.getElementById('btn-control');
  const navMonitorBtn = document.getElementById('btn-monitor');
  const controlPanel = document.getElementById('control-panel');
  const monitorPanel = document.getElementById('monitor-panel');

  // Control panel elements
  const topicInput = document.getElementById('topicInput');
  const setTopicBtn = document.getElementById('setTopicBtn');
  const activeTopicLabel = document.getElementById('active-topic');
  const controls = document.getElementById('controls');
  const statusEl = document.getElementById('status');

  // Monitoring page elements
  const addTopicBtn = document.getElementById('addTopicBtn');
  const newTopicInput = document.getElementById('newTopicInput');
  const sensorPanelsContainer = document.getElementById('sensor-panels');

  // Initialize ROS connection
  const ros = new ROSLIB.Ros({
    url: 'ws://localhost:9090'
  });

  let currentTopicName = topicInput.value.trim() || '/cmd_vel';
  let cmdTopic = null;

  // Maintain map of monitored topics to subscription and chart info
  const subscribedTopics = {};

  // --- NAVIGATION ---
  function showPanel(panelName) {
    if(panelName === 'control') {
      controlPanel.classList.add('visible');
      monitorPanel.classList.remove('visible');
      navControlBtn.classList.add('active');
      navControlBtn.setAttribute('aria-selected', 'true');
      navMonitorBtn.classList.remove('active');
      navMonitorBtn.setAttribute('aria-selected', 'false');
      controlPanel.setAttribute('aria-hidden', 'false');
      monitorPanel.setAttribute('aria-hidden', 'true');
    } else if(panelName === 'monitor') {
      monitorPanel.classList.add('visible');
      controlPanel.classList.remove('visible');
      navMonitorBtn.classList.add('active');
      navMonitorBtn.setAttribute('aria-selected', 'true');
      navControlBtn.classList.remove('active');
      navControlBtn.setAttribute('aria-selected', 'false');
      monitorPanel.setAttribute('aria-hidden', 'false');
      controlPanel.setAttribute('aria-hidden', 'true');
    }
  }
  navControlBtn.addEventListener('click', () => showPanel('control'));
  navMonitorBtn.addEventListener('click', () => showPanel('monitor'));

  // --- CONTROL PANEL FUNCTIONS ---

  // Create or update ROS topic publisher for cmd_vel or custom topic
  function updateTopic(name) {
    if (cmdTopic) {
      try {
        cmdTopic.unadvertise();
      } catch(e) {
        // Ignore if not advertised, just be safe
      }
    }
    currentTopicName = name;
    cmdTopic = new ROSLIB.Topic({
      ros,
      name,
      messageType: 'geometry_msgs/msg/Twist'
    });
    activeTopicLabel.textContent = name;
    statusEl.textContent = `Target topic set: ${name}`;
  }

  setTopicBtn.addEventListener('click', () => {
    const newTopic = topicInput.value.trim();
    if (newTopic.length === 0) {
      statusEl.textContent = 'Please enter a topic name';
      return;
    }
    updateTopic(newTopic);
  });

  // Send cmd_vel movement command
  function sendCmd(direction) {
    if (!cmdTopic) {
      statusEl.textContent = 'No topic selected';
      return;
    }
    let linearX = 0;
    let angularZ = 0;

    switch (direction) {
      case 'forward': linearX = 0.3; break;
      case 'backward': linearX = -0.3; break;
      case 'left': angularZ = 1.0; break;
      case 'right': angularZ = -1.0; break;
      case 'stop': linearX = 0; angularZ = 0; break;
    }

    const twistMsg = new ROSLIB.Message({
      linear: { x: linearX, y: 0, z: 0 },
      angular: { x: 0, y: 0, z: angularZ }
    });

    cmdTopic.publish(twistMsg);
    statusEl.textContent = `Sent "${direction}" to ${currentTopicName}`;
  }

  // Attach control buttons event listeners
  controls.querySelectorAll('button').forEach(btn => {
    btn.addEventListener('click', () => {
      const cmd = btn.getAttribute('data-cmd');
      sendCmd(cmd);
    });
  });

  // ROS connection events
  ros.on('connection', () => {
    statusEl.textContent = 'Connected to rosbridge!';
    updateTopic(currentTopicName);
  });

  ros.on('error', (error) => {
    statusEl.textContent = `Error: ${error}`;
  });

  ros.on('close', () => {
    statusEl.textContent = 'Connection to rosbridge closed';
  });

  // --- MONITORING PANEL FUNCTIONS ---

  // Subscribe to a ROS topic and create dynamic sensor panel
  function subscribeToTopic(topicName) {
    if (subscribedTopics[topicName]) {
      alert(`Already subscribed to topic ${topicName}`);
      return;
    }

    // Create panel container
    const panelDiv = document.createElement('div');
    panelDiv.classList.add('sensor-panel');

    const header = document.createElement('h3');
    header.textContent = `Topic: ${topicName}`;
    panelDiv.appendChild(header);

    // Canvas or image will be appended below depending on topic type
    let chartCanvas = null;
    let chart = null;

    // Determine if the topic name includes 'image' indicating image topic
    if (topicName.toLowerCase().includes('image')) {
      // Image topic panel
      const img = document.createElement('img');
      img.style.width = '100%';
      img.style.borderRadius = '6px';
      img.alt = `Image stream from topic ${topicName}`;
      panelDiv.appendChild(img);

      // Subscribe to ROS image topic (sensor_msgs/msg/Image)
      const imgSub = new ROSLIB.Topic({
        ros,
        name: topicName,
        messageType: 'sensor_msgs/msg/Image',
      });

      imgSub.subscribe((msg) => {
        // This assumes base64 encoded jpeg in msg.data — adapt as needed for your setup
        if (msg.data) {
          img.src = `data:image/jpeg;base64,${msg.data}`;
        }
      });

      subscribedTopics[topicName] = { subscriber: imgSub, panel: panelDiv };
    } else {
      // Numeric / sensor data: create canvas and Chart.js graph
      chartCanvas = document.createElement('canvas');
      chartCanvas.height = 180;
      panelDiv.appendChild(chartCanvas);

      chart = new Chart(chartCanvas.getContext('2d'), {
        type: 'line',
        data: {
          labels: [],
          datasets: [],
        },
        options: {
          animation: false,
          responsive: true,
          maintainAspectRatio: false,
          scales: {
            x: {
              title: { display: true, text: 'Time' },
              ticks: { maxRotation: 0 },
            },
            y: {
              title: { display: true, text: 'Value' },
            }
          },
          plugins: {
            legend: { position: 'bottom' }
          }
        }
      });

      // Subscribe to topic, assuming generic message type that contains numeric fields (use sensor_msgs/msg/Imu as default)
      const sub = new ROSLIB.Topic({
        ros,
        name: topicName,
        messageType: 'sensor_msgs/msg/Imu' // Change as needed
      });

      let initialized = false;

      sub.subscribe((msg) => {
        if (!initialized) {
          createDatasetsForMsg(chart, msg);
          initialized = true;
        }
        updateChartWithMsg(chart, msg);
      });

      subscribedTopics[topicName] = { subscriber: sub, chart, panel: panelDiv };
    }

    sensorPanelsContainer.appendChild(panelDiv);
  }

  // Recursively create datasets for each numeric msg field (nested included)
  function createDatasetsForMsg(chart, msg, parentKey = '') {
    for (const key in msg) {
      const fullKey = parentKey ? `${parentKey}.${key}` : key;
      if (typeof msg[key] === 'number') {
        chart.data.datasets.push({
          label: fullKey,
          data: [],
          borderColor: randomColor(),
          fill: false,
          lineTension: 0.05
        });
      } else if (typeof msg[key] === 'object' && msg[key] !== null) {
        createDatasetsForMsg(chart, msg[key], fullKey);
      }
    }
    chart.update();
  }

  // Helper to get nested value from message by dot-notation key
  function getValueFromKey(msg, key) {
    const parts = key.split('.');
    let val = msg;
    for (const part of parts) {
      if (val && val.hasOwnProperty(part)) {
        val = val[part];
      } else {
        return null;
      }
    }
    return (typeof val === 'number') ? val : null;
  }

  // Append new data point for each dataset; limit to 50 points on x axis
  function updateChartWithMsg(chart, msg) {
    const timeLabel = new Date().toLocaleTimeString();

    if (chart.data.labels.length > 50) {
      chart.data.labels.shift();
      chart.data.datasets.forEach(ds => ds.data.shift());
    }
    chart.data.labels.push(timeLabel);

    chart.data.datasets.forEach(ds => {
      const value = getValueFromKey(msg, ds.label);
      ds.data.push(value !== null ? value : NaN);
    });

    chart.update();
  }

  // Generate pastel random color for lines
  function randomColor() {
    const r = Math.floor(100 + Math.random() * 155);
    const g = Math.floor(100 + Math.random() * 155);
    const b = Math.floor(100 + Math.random() * 155);
    return `rgb(${r},${g},${b})`;
  }

  // Add Topic button handler
  addTopicBtn.addEventListener('click', () => {
    const userInput = newTopicInput.value.trim();
    if (!userInput) {
      alert('Please enter one or more topics');
      return;
    }
    // Split by comma, clean spaces, filter empty strings
    const topics = userInput.split(',').map(t => t.trim()).filter(t => t);
    topics.forEach(t => subscribeToTopic(t));
    newTopicInput.value = '';
  });

  // Initialize site showing control panel
  showPanel('control');
});

const rosnodejs = require('rosnodejs');

module.exports = {
  name: 'ros2Subscriber',
  displayName: 'ROS 2 Subscriber',
  description: 'Subscribes to ROS 2 topics and receives messages',
  version: 1,
  icon: 'robot',
  group: ['input'],
  defaultVersion: 1,

  inputs: {
    main: {
      type: 'main',
      displayName: '',
      group: ['main'],
      sortKey: 0,
    },
  },

  outputs: {
    main: {
      type: 'main',
      displayName: 'Message',
      baseType: 'resource',
      group: ['main'],
      sortKey: 0,
    },
    error: {
      type: 'main',
      displayName: 'Error',
      baseType: 'resource',
      group: ['main'],
      sortKey: 1,
    },
  },

  properties: [
    {
      displayName: 'Topic',
      name: 'topic',
      type: 'string',
      default: '/scan',
      placeholder: '/scan',
      description: 'ROS 2 topic to subscribe to',
      required: true,
    },
    {
      displayName: 'Message Type',
      name: 'messageType',
      type: 'options',
      options: [
        { name: 'sensor_msgs/LaserScan', value: 'sensor_msgs/LaserScan' },
        { name: 'geometry_msgs/Twist', value: 'geometry_msgs/Twist' },
        { name: 'std_msgs/String', value: 'std_msgs/String' },
        { name: 'std_msgs/Float64', value: 'std_msgs/Float64' },
      ],
      default: 'sensor_msgs/LaserScan',
      description: 'ROS message type to expect',
      required: true,
    },
    {
      displayName: 'Timeout (seconds)',
      name: 'timeout',
      type: 'number',
      default: 5,
      placeholder: '5',
      description: 'Maximum time to wait for a message (seconds)',
      required: true,
    },
    {
      displayName: 'Wait for New Message',
      name: 'waitForNew',
      type: 'boolean',
      default: true,
      description: 'Whether to wait for a new message or return the last received one',
    },
  ],

  async execute(this: any) {
    try {
      // Initialize ROS node if not already done
      if (!this.nodeHandle) {
        this.nodeHandle = await rosnodejs.initNode('n8n_ros2_subscriber', {
          rosMasterUri: process.env.ROS_MASTER_URI || 'http://localhost:11311'
        });
      }

      const topic = this.getNodeParameter('topic', '/scan') as string;
      const messageType = this.getNodeParameter('messageType', 'sensor_msgs/LaserScan') as string;
      const timeout = (this.getNodeParameter('timeout', 5) as number) * 1000; // Convert to ms
      const waitForNew = this.getNodeParameter('waitForNew', true) as boolean;

      // Create subscriber if topic changed or doesn't exist
      if (!this.subscriber || this.topic !== topic || this.messageType !== messageType) {
        if (this.subscriber) {
          this.subscriber.shutdown();
        }
        this.subscriber = this.nodeHandle.subscribe(topic, messageType);
        this.topic = topic;
        this.messageType = messageType;
      }

      // Return last received message or wait for new one
      if (!waitForNew && this.lastMessage) {
        const message = this.lastMessage;
        this.lastMessage = null; // Clear after use
        return this.prepareOutputData({
          json: {
            success: true,
            topic: topic,
            messageType: messageType,
            message: message,
            timestamp: new Date().toISOString()
          }
        });
      }

      // Wait for new message
      return new Promise((resolve, reject) => {
        const timeoutId = setTimeout(() => {
          reject(new Error(`Timeout waiting for message on topic ${topic}`));
        }, timeout);

        this.subscriber.on('message', (message) => {
          clearTimeout(timeoutId);
          this.lastMessage = message;
          resolve(this.prepareOutputData({
            json: {
              success: true,
              topic: topic,
              messageType: messageType,
              message: message,
              timestamp: new Date().toISOString()
            }
          }));
        });
      });

    } catch (error) {
      console.error('ROS 2 Subscriber Error:', error);
      throw new Error(`ROS 2 Subscriber failed: ${error.message}`);
    }
  },
};

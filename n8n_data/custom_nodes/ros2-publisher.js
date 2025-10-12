const rosnodejs = require('rosnodejs');

module.exports = {
  name: 'ros2Publisher',
  codex: {
    categories: ['Communication'],
    subcategory: 'ROS 2',
    alias: ['ROS2', 'Robot'],
  },
  displayName: 'ROS 2 Publisher',
  description: 'Publishes messages to ROS 2 topics',
  version: 1,
  icon: 'robot',
  group: ['transform'],
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
      displayName: 'Success',
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
      default: '/cmd_vel',
      placeholder: '/cmd_vel',
      description: 'ROS 2 topic to publish to',
      required: true,
    },
    {
      displayName: 'Message Type',
      name: 'messageType',
      type: 'options',
      options: [
        { name: 'geometry_msgs/Twist', value: 'geometry_msgs/Twist' },
        { name: 'std_msgs/String', value: 'std_msgs/String' },
        { name: 'std_msgs/Float64', value: 'std_msgs/Float64' },
      ],
      default: 'geometry_msgs/Twist',
      description: 'ROS message type',
      required: true,
    },
    {
      displayName: 'Linear X',
      name: 'linearX',
      type: 'number',
      default: 0,
      placeholder: '0',
      description: 'Linear velocity in X direction (m/s)',
      displayOptions: {
        show: {
          messageType: ['geometry_msgs/Twist'],
        },
      },
    },
    {
      displayName: 'Linear Y',
      name: 'linearY',
      type: 'number',
      default: 0,
      placeholder: '0',
      description: 'Linear velocity in Y direction (m/s)',
      displayOptions: {
        show: {
          messageType: ['geometry_msgs/Twist'],
        },
      },
    },
    {
      displayName: 'Angular Z',
      name: 'angularZ',
      type: 'number',
      default: 0,
      placeholder: '0',
      description: 'Angular velocity around Z axis (rad/s)',
      displayOptions: {
        show: {
          messageType: ['geometry_msgs/Twist'],
        },
      },
    },
    {
      displayName: 'String Data',
      name: 'stringData',
      type: 'string',
      default: '',
      placeholder: 'Hello ROS',
      description: 'String message data',
      displayOptions: {
        show: {
          messageType: ['std_msgs/String'],
        },
      },
    },
    {
      displayName: 'Float Data',
      name: 'floatData',
      type: 'number',
      default: 0,
      placeholder: '0.0',
      description: 'Float message data',
      displayOptions: {
        show: {
          messageType: ['std_msgs/Float64'],
        },
      },
    },
  ],

  async execute(this: any) {
    try {
      // Initialize ROS node if not already done
      if (!this.nodeHandle) {
        this.nodeHandle = await rosnodejs.initNode('n8n_ros2_bridge', {
          rosMasterUri: process.env.ROS_MASTER_URI || 'http://localhost:11311'
        });
      }

      const topic = this.getNodeParameter('topic', '/cmd_vel') as string;
      const messageType = this.getNodeParameter('messageType', 'geometry_msgs/Twist') as string;

      // Create publisher if topic changed or doesn't exist
      if (!this.publisher || this.topic !== topic || this.messageType !== messageType) {
        if (this.publisher) {
          this.publisher.shutdown();
        }
        this.publisher = this.nodeHandle.advertise(topic, messageType);
        this.topic = topic;
        this.messageType = messageType;
      }

      // Build message based on type
      let message = {};

      if (messageType === 'geometry_msgs/Twist') {
        message = {
          linear: {
            x: this.getNodeParameter('linearX', 0) as number,
            y: this.getNodeParameter('linearY', 0) as number,
            z: 0
          },
          angular: {
            x: 0,
            y: 0,
            z: this.getNodeParameter('angularZ', 0) as number
          }
        };
      } else if (messageType === 'std_msgs/String') {
        message = {
          data: this.getNodeParameter('stringData', '') as string
        };
      } else if (messageType === 'std_msgs/Float64') {
        message = {
          data: this.getNodeParameter('floatData', 0) as number
        };
      }

      // Publish message
      this.publisher.publish(message);

      return this.prepareOutputData({
        json: {
          success: true,
          topic: topic,
          messageType: messageType,
          message: message,
          timestamp: new Date().toISOString()
        }
      });

    } catch (error) {
      console.error('ROS 2 Publisher Error:', error);
      throw new Error(`ROS 2 Publisher failed: ${error.message}`);
    }
  },
};

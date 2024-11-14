using UnityEngine;

namespace RosSharp.RosBridgeClient
{
    public class Int32MultiArrayPublisher : UnityPublisher<MessageTypes.Std.Int32MultiArray>
    {
        private MessageTypes.Std.Int32MultiArray message = new MessageTypes.Std.Int32MultiArray();

        protected override void Start()
        {
            base.Start();
            InitializeMessage();
        }
        private void InitializeMessage()
        {
            message = new MessageTypes.Std.Int32MultiArray
            {
                data = DoubleData
            };
        }
        private void FixedUpdate()
        {
            UpdateMessage();
        }
        public void UpdateMessage()
        {
            message.data = DoubleData;
            Publish(message);
            Debug.Log("Published message: " + message.data);
        }
    }
}

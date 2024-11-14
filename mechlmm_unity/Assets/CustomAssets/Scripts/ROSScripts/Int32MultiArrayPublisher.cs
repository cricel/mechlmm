using UnityEngine;

namespace RosSharp.RosBridgeClient
{
    public class Int32MultiArrayPublisher : UnityPublisher<MessageTypes.Std.Int32MultiArray>
    {
        private MessageTypes.Std.Int32MultiArray message = new MessageTypes.Std.Int32MultiArray();

        protected override void Start()
        {
            base.Start();
            message.data = new int[] { -1, -1 };
        }

        private void FixedUpdate()
        {
            UpdateMessage();
        }
        public void UpdateMessage()
        {
            // message.data = DoubleData;
            // Publish(message);
            // Debug.Log("Published message: " + message.data);
        }
    }
}

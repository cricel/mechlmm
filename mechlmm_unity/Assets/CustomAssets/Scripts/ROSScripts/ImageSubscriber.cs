using UnityEngine;

namespace RosSharp.RosBridgeClient
{
    public class ImageSubscriber : UnitySubscriber<MessageTypes.Sensor.Image>
    {
        public bool isMessageReceived;
        private byte[] messageData;
        
        protected override void ReceiveMessage(MessageTypes.Sensor.Image message)
        {
            messageData = message.data;
            isMessageReceived = true;
        }
        
        protected override void Start()
        {
            base.Start();
        }

        void FixedUpdate()
        {
            Debug.Log("Received message: " + messageData);
        }

        public byte[] GetImageData(){
            return messageData;
        }
    }
}
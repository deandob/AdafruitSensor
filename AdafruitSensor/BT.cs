using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Windows.Devices.Bluetooth.Rfcomm;
using Windows.Devices.Enumeration;
using Windows.Networking.Sockets;
using Windows.Storage.Streams;

// HM10 module - Name: HMSoft; Baud: 9600, N, 8, 1; Pin code: 000000; transmit 
// https://www.pridopia.co.uk/pi-doc/BT4.0-HM-10-Serial_Port_BLE_Module_Master_Slave.pdf
// Blinks when not paired, full green LED when paired
// Add bluetooth capabilities to appxmanifest

namespace AdafruitSensor    
{
    class BT
    {
        private StreamSocket _socket;

        private RfcommDeviceService _service;

        public async void Send()
        {
            var noOfCharsSent = await Send("123");
        }

        private async Task<uint> Send(string msg)
        {
            try
            {
                var writer = new DataWriter(_socket.OutputStream);

                writer.WriteString(msg);

                // Launch an async task to 
                //complete the write operation
                var store = writer.StoreAsync().AsTask();

                return await store;
            }
            catch (Exception ex)
            {
                //tbError.Text = ex.Message;

                return 0;
            }
        }

        public async void Connect()
        {
            try
            {
                var devices = await DeviceInformation.FindAllAsync(
                    RfcommDeviceService.GetDeviceSelector(
                    RfcommServiceId.SerialPort));

                var device = devices.Single(x => x.Name == "HMSoft");

                _service = await RfcommDeviceService.FromIdAsync(
                device.Id);

                _socket = new StreamSocket();

                await _socket.ConnectAsync(
               _service.ConnectionHostName,
               _service.ConnectionServiceName,
               SocketProtectionLevel.
               BluetoothEncryptionAllowNullAuthentication);
            }
            catch (Exception ex)
            {
                //tbError.Text = ex.Message;
            }
        }

        private async void Disconnect()
        {
            try
            {
                await _socket.CancelIOAsync();
                _socket.Dispose();
                _socket = null;
                _service.Dispose();
                _service = null;
            }
            catch (Exception ex)
            {
                //tbError.Text = ex.Message;
            }
        }
    }
}

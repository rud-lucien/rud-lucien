// See https://aka.ms/new-console-template for more information

using ids_peak; // Replace with the correct namespace from the DLL.

static void Main(string[] args)
{
    // Initialize the API
    peak.Library.Initialize();

    // Create a camera manager instance
    var cameraManager = new peak.core.CameraManager();

    // Open the first available camera
    var cameraList = cameraManager.UpdateAndGet();
    if (cameraList.Count == 0)
    {
        Console.WriteLine("No camera found.");
        return;
    }
    var camera = cameraManager.OpenCamera(cameraList[0]);
    Console.WriteLine("Camera opened: " + camera.Properties.Name);
}

camera.NodeMap["AcquisitionMode"].Value = "Continuous";

camera.NodeMap["ExposureTime"].Value = 20000.0; // Example: 20ms
camera.NodeMap["Width"].Value = 1920; // Set width
camera.NodeMap["Height"].Value = 1080; // Set height

camera.AcquisitionStart();

using (var stream = camera.CreateStream())
{
    stream.StartAcquisition();
    for (int i = 0; i < 300; i++) // Example: 300 frames
    {
        var image = stream.WaitForFrame(1000); // Timeout in ms
        if (image != null)
        {
            Console.WriteLine("Captured frame: " + i);
            // Save the frame or add to video encoding pipeline
            SaveImage(image); // Implement your image saving logic
        }
    }
    stream.StopAcquisition();
}

camera.AcquisitionStop();

string folderPath = @"C:\VideoCapture\";
string fileName = $"ProcessVideo_{DateTime.Now:yyyyMMdd_HHmmss}.mp4";
string fullPath = Path.Combine(folderPath, fileName);

camera.Dispose();
peak.Library.Close();

using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Navigation;
using System.Windows.Shapes;
using Microsoft.Kinect;
using System.IO;
using System.Diagnostics;
using System.Windows.Media.Media3D;


namespace StillKicking
{


    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window
    {
        /// <summary>
        /// Width of output drawing
        /// </summary>
        private const float RenderWidth = 640.0f;

        /// <summary>
        /// Height of our output drawing
        /// </summary>
        private const float RenderHeight = 480.0f;

        /// <summary>
        /// Thickness of drawn joint lines
        /// </summary>
        private const double JointThickness = 3;

        /// <summary>
        /// Thickness of body center ellipse
        /// </summary>
        private const double BodyCenterThickness = 10;

        /// <summary>
        /// Thickness of clip edge rectangles
        /// </summary>
        private const double ClipBoundsThickness = 10;

        /// <summary>
        /// Brush used to draw skeleton center point
        /// </summary>
        private readonly Brush centerPointBrush = Brushes.Blue;

        /// <summary>
        /// Brush used for drawing joints that are currently tracked
        /// </summary>
        private readonly Brush trackedJointBrush = new SolidColorBrush(Color.FromArgb(255, 68, 192, 68));

        /// <summary>
        /// Brush used for drawing joints that are currently inferred
        /// </summary>        
        private readonly Brush inferredJointBrush = Brushes.Yellow;

        /// <summary>
        /// Pen used for drawing bones that are currently tracked
        /// </summary>
        private readonly Pen trackedBonePen = new Pen(Brushes.Green, 6);

        /// <summary>
        /// Pen used for drawing bones that are currently out of tolerance range for flex
        /// </summary>
        private readonly Pen outOfRangeBonePen = new Pen(Brushes.Red, 6);

        /// <summary>
        /// Pen used for drawing bones that are currently inferred
        /// </summary>        
        private readonly Pen inferredBonePen = new Pen(Brushes.Gray, 1);

        /// <summary>
        /// Active Kinect sensor
        /// </summary>
        private KinectSensor sensor;

        /// <summary>
        /// Drawing group for skeleton rendering output
        /// </summary>
        private DrawingGroup drawingGroup;

        /// <summary>
        /// Drawing image that we will display
        /// </summary>
        private DrawingImage imageSource;

        /// <summary>
        /// Initializes a new instance of the MainWindow class.
        /// </summary>
        public MainWindow()
        {
            InitializeComponent();
        }

        /// <summary>
        /// Draws indicators to show which edges are clipping skeleton data
        /// </summary>
        /// <param name="skeleton">skeleton to draw clipping information for</param>
        /// <param name="drawingContext">drawing context to draw to</param>
        private static void RenderClippedEdges(Skeleton skeleton, DrawingContext drawingContext)
        {
            if (skeleton.ClippedEdges.HasFlag(FrameEdges.Bottom))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(0, RenderHeight - ClipBoundsThickness, RenderWidth, ClipBoundsThickness));
            }

            if (skeleton.ClippedEdges.HasFlag(FrameEdges.Top))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(0, 0, RenderWidth, ClipBoundsThickness));
            }

            if (skeleton.ClippedEdges.HasFlag(FrameEdges.Left))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(0, 0, ClipBoundsThickness, RenderHeight));
            }

            if (skeleton.ClippedEdges.HasFlag(FrameEdges.Right))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(RenderWidth - ClipBoundsThickness, 0, ClipBoundsThickness, RenderHeight));
            }
        }

        /// <summary>
        /// Execute startup tasks
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void WindowLoaded(object sender, RoutedEventArgs e)
        {
            // Create the drawing group we'll use for drawing
            this.drawingGroup = new DrawingGroup();

            // Create an image source that we can use in our image control
            this.imageSource = new DrawingImage(this.drawingGroup);

            // Display the drawing using our image control
            Image.Source = this.imageSource;

            // Look through all sensors and start the first connected one.
            // This requires that a Kinect is connected at the time of app startup.
            // To make your app robust against plug/unplug, 
            // it is recommended to use KinectSensorChooser provided in Microsoft.Kinect.Toolkit (See components in Toolkit Browser).
            foreach (var potentialSensor in KinectSensor.KinectSensors)
            {
                if (potentialSensor.Status == KinectStatus.Connected)
                {
                    this.sensor = potentialSensor;
                    break;
                }
            }

            if (null != this.sensor)
            {
                // Turn on the skeleton stream to receive skeleton frames
                this.sensor.SkeletonStream.Enable();

                // Add an event handler to be called whenever there is new color frame data
                this.sensor.SkeletonFrameReady += this.SensorSkeletonFrameReady;

                // Start the sensor!
                try
                {
                    this.sensor.Start();
                }
                catch (IOException)
                {
                    this.sensor = null;
                }
            }

            if (null == this.sensor)
            {
                this.statusBarText.Text = Properties.Resources.NoKinectReady;
            }
        }

        /// <summary>
        /// Execute shutdown tasks
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void WindowClosing(object sender, System.ComponentModel.CancelEventArgs e)
        {
            if (null != this.sensor)
            {
                this.sensor.Stop();
            }
        }

        /// <summary>
        /// Event handler for Kinect sensor's SkeletonFrameReady event
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void SensorSkeletonFrameReady(object sender, SkeletonFrameReadyEventArgs e)
        {
            Skeleton[] skeletons = new Skeleton[0];

            using (SkeletonFrame skeletonFrame = e.OpenSkeletonFrame())
            {
                if (skeletonFrame != null)
                {
                    skeletons = new Skeleton[skeletonFrame.SkeletonArrayLength];
                    skeletonFrame.CopySkeletonDataTo(skeletons);
                }
            }

            using (DrawingContext dc = this.drawingGroup.Open())
            {
                // Draw a transparent background to set the render size
                dc.DrawRectangle(Brushes.Black, null, new Rect(0.0, 0.0, RenderWidth, RenderHeight));

                //  Debug.WriteLine(skeletons.Length);

                if (skeletons.Length != 0)
                {
                    foreach (Skeleton skel in skeletons)
                    {
                        RenderClippedEdges(skel, dc);

                        if (skel.TrackingState == SkeletonTrackingState.Tracked)
                        {
                            this.DrawBonesAndJoints(skel, dc);
                        }
                        else if (skel.TrackingState == SkeletonTrackingState.PositionOnly)
                        {
                            dc.DrawEllipse(
                            this.centerPointBrush,
                            null,
                            this.SkeletonPointToScreen(skel.Position),
                            BodyCenterThickness,
                            BodyCenterThickness);
                        }
                    }
                }

                // prevent drawing outside of our render area
                this.drawingGroup.ClipGeometry = new RectangleGeometry(new Rect(0.0, 0.0, RenderWidth, RenderHeight));
            }
        }

        /// <summary>
        /// Draws a skeleton's bones and joints
        /// </summary>
        /// <param name="skeleton">skeleton to draw</param>
        /// <param name="drawingContext">drawing context to draw to</param>
        private void DrawBonesAndJoints(Skeleton skeleton, DrawingContext drawingContext)
        {
            // Render Torso
            this.DrawBone(skeleton, drawingContext, JointType.Head, JointType.ShoulderCenter);
            this.DrawBone(skeleton, drawingContext, JointType.ShoulderCenter, JointType.ShoulderLeft);
            this.DrawBone(skeleton, drawingContext, JointType.ShoulderCenter, JointType.ShoulderRight);
            this.DrawBone(skeleton, drawingContext, JointType.ShoulderCenter, JointType.Spine);
            this.DrawBone(skeleton, drawingContext, JointType.Spine, JointType.HipCenter);
            this.DrawBone(skeleton, drawingContext, JointType.HipCenter, JointType.HipLeft);
            this.DrawBone(skeleton, drawingContext, JointType.HipCenter, JointType.HipRight);

            // Left Arm bend angle
            var leftAngle = GetJointAngle(skeleton, JointType.ShoulderLeft, JointType.ElbowLeft, JointType.WristLeft);
            Debug.WriteLine("Left arm angle:{0}", leftAngle);

            var rightAngle = GetJointAngle(skeleton, JointType.ShoulderLeft, JointType.ElbowLeft, JointType.WristLeft);
            Debug.WriteLine("Right arm angle:{0}", rightAngle);

            var leftShoulder = GetJointAngle(skeleton, JointType.ShoulderCenter, JointType.ShoulderLeft, JointType.ElbowLeft);
            Debug.WriteLine("Left shoulder angle:{0}", leftShoulder);

            var rightShoulder = GetJointAngle(skeleton, JointType.ShoulderCenter, JointType.ShoulderRight, JointType.ElbowRight);
            Debug.WriteLine("Right shoulder angle:{0}", rightShoulder);
            //see if angle isn't right, if so, pass in color to draw bone or bool value

            if ((leftAngle > 140 && leftAngle < 180) && (rightAngle > 140 && rightAngle < 180))
            {
                picArmsUp.Visibility = Visibility.Visible;
                picArmsDown.Visibility = Visibility.Collapsed;
            }

            if ((leftAngle > 10 && leftAngle < 50) && (rightAngle > 10 && rightAngle < 50)) 
            {
                picArmsUp.Visibility = Visibility.Collapsed;
                picArmsDown.Visibility = Visibility.Visible;
            }

            if ((leftShoulder > 130 && leftShoulder < 150) && (rightShoulder > 130 && rightShoulder < 150))
            {
                greenRec.Visibility = Visibility.Visible;
                redRec.Visibility = Visibility.Collapsed;
            }
            else
            {
                greenRec.Visibility = Visibility.Collapsed;
                redRec.Visibility = Visibility.Visible;
            }



            this.DrawBone(skeleton, drawingContext, JointType.ShoulderLeft, JointType.ElbowLeft);
            this.DrawBone(skeleton, drawingContext, JointType.ElbowLeft, JointType.WristLeft);
            this.DrawBone(skeleton, drawingContext, JointType.WristLeft, JointType.HandLeft);
             
            

            // Right Arm
            this.DrawBone(skeleton, drawingContext, JointType.ShoulderRight, JointType.ElbowRight); 
            this.DrawBone(skeleton, drawingContext, JointType.ElbowRight, JointType.WristRight);
            this.DrawBone(skeleton, drawingContext, JointType.WristRight, JointType.HandRight);

            // Left Leg
            this.DrawBone(skeleton, drawingContext, JointType.HipLeft, JointType.KneeLeft);
            this.DrawBone(skeleton, drawingContext, JointType.KneeLeft, JointType.AnkleLeft);
            this.DrawBone(skeleton, drawingContext, JointType.AnkleLeft, JointType.FootLeft);

            // Right Leg
            this.DrawBone(skeleton, drawingContext, JointType.HipRight, JointType.KneeRight);
            this.DrawBone(skeleton, drawingContext, JointType.KneeRight, JointType.AnkleRight);
            this.DrawBone(skeleton, drawingContext, JointType.AnkleRight, JointType.FootRight);

            // Render Joints
            foreach (Joint joint in skeleton.Joints)
            {
                Brush drawBrush = null;

                if (joint.TrackingState == JointTrackingState.Tracked)
                {
                    drawBrush = this.trackedJointBrush;
                }
                else if (joint.TrackingState == JointTrackingState.Inferred)
                {
                    drawBrush = this.inferredJointBrush;
                }

                if (drawBrush != null)
                {
                    drawingContext.DrawEllipse(drawBrush, null, this.SkeletonPointToScreen(joint.Position), JointThickness, JointThickness);
                }
            }
        }

        private double GetJointAngle(Skeleton skeleton, JointType jointType1, JointType jointType2, JointType jointType3)
        {

            Joint joint0 = skeleton.Joints[jointType1];
            Joint joint1 = skeleton.Joints[jointType2];
            Joint joint2 = skeleton.Joints[jointType3];


            var joint0ToJoint1 = new Vector3D(joint0.Position.X - joint1.Position.X,
                                        joint0.Position.Y - joint1.Position.Y,
                                        joint0.Position.Z - joint1.Position.Z);

            var joint1ToJoint2 = new Vector3D(joint2.Position.X - joint1.Position.X,
                                        joint2.Position.Y - joint1.Position.Y,
                                        joint2.Position.Z - joint1.Position.Z);


            joint0ToJoint1.Normalize();
            joint1ToJoint2.Normalize();

            var dotProduct = Vector3D.DotProduct(joint0ToJoint1, joint1ToJoint2);
            var crossProduct = Vector3D.CrossProduct(joint0ToJoint1, joint1ToJoint2);

            var angleFormed = Math.Atan2(crossProduct.Length, dotProduct);
            double angleInDegree = angleFormed * (180 / Math.PI);
            return Math.Round(angleInDegree, 2);
        }

        /// <summary>
        /// Maps a SkeletonPoint to lie within our render space and converts to Point
        /// </summary>
        /// <param name="skelpoint">point to map</param>
        /// <returns>mapped point</returns>
        private Point SkeletonPointToScreen(SkeletonPoint skelpoint)
        {
            // Convert point to depth space.  
            // We are not using depth directly, but we do want the points in our 640x480 output resolution.
            DepthImagePoint depthPoint = this.sensor.CoordinateMapper.MapSkeletonPointToDepthPoint(skelpoint, DepthImageFormat.Resolution640x480Fps30);
            return new Point(depthPoint.X, depthPoint.Y);
        }

        /// <summary>
        /// Draws a bone line between two joints
        /// </summary>
        /// <param name="skeleton">skeleton to draw bones from</param>
        /// <param name="drawingContext">drawing context to draw to</param>
        /// <param name="jointType0">joint to start drawing from</param>
        /// <param name="jointType1">joint to end drawing at</param>
        /// <param name="outofRange"></param>
        private void DrawBone(Skeleton skeleton, DrawingContext drawingContext, JointType jointType0, JointType jointType1, bool outofRange=false)
        {
            Joint joint0 = skeleton.Joints[jointType0];
            Joint joint1 = skeleton.Joints[jointType1];


            // If we can't find either of these joints, exit
            if (joint0.TrackingState == JointTrackingState.NotTracked ||
                joint1.TrackingState == JointTrackingState.NotTracked)
            {
                return;
            }

            // Don't draw if both points are inferred
            if (joint0.TrackingState == JointTrackingState.Inferred &&
                joint1.TrackingState == JointTrackingState.Inferred)
            {
                return;
            }

            // We assume all drawn bones are inferred unless BOTH joints are tracked
            Pen drawPen = this.inferredBonePen;
            if (joint0.TrackingState == JointTrackingState.Tracked && joint1.TrackingState == JointTrackingState.Tracked)
            {
                if (outofRange)
                {
                    drawPen = outOfRangeBonePen;
                }
                else
                {
                    drawPen = this.trackedBonePen;
                }
            }

            drawingContext.DrawLine(drawPen, this.SkeletonPointToScreen(joint0.Position), this.SkeletonPointToScreen(joint1.Position));
        }

        /// <summary>
        /// Handles the checking or unchecking of the seated mode combo box
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void CheckBoxSeatedModeChanged(object sender, RoutedEventArgs e)
        {
            if (null != this.sensor)
            {
                if (this.checkBoxSeatedMode.IsChecked.GetValueOrDefault())
                {
                    this.sensor.SkeletonStream.TrackingMode = SkeletonTrackingMode.Seated;
                }
                else
                {
                    this.sensor.SkeletonStream.TrackingMode = SkeletonTrackingMode.Default;
                }
            }
        }

    }
    ///// <summary>
    ///// Interaction logic for MainWindow.xaml
    ///// </summary>
    //public partial class MainWindow : Window
    //{
    //    public MainWindow()
    //    {
    //        InitializeComponent();
    //    }

    //    bool closing = false;
    //    const int skeletonCount = 6;
    //    Skeleton[] allSkeletons = new Skeleton[skeletonCount];
    //    KinectSensor _sensor;



    //    private void Window_Loaded(object sender, RoutedEventArgs e)
    //    {
    //        if (KinectSensor.KinectSensors.Count>0)
    //        {
    //            _sensor = KinectSensor.KinectSensors[0];

    //            if (_sensor.Status == KinectStatus.Connected)
    //            {
    //                _sensor.ColorStream.Enable();
    //                _sensor.DepthStream.Enable();
    //                _sensor.SkeletonStream.Enable();
    //                _sensor.AllFramesReady += new EventHandler<AllFramesReadyEventArgs>(_sensor_AllFramesReady);
    //                _sensor.Start();

    //            }
    //        }
    //    }

    //    void _sensor_AllFramesReady(object sender, AllFramesReadyEventArgs e)
    //    {


    //        //throw new NotImplementedException();
    //        using (ColorImageFrame colorFrame = e.OpenColorImageFrame())
    //        {
    //            if (colorFrame == null)
    //            {
    //                return;
    //            }

    //            byte[] colorPixels = new byte[colorFrame.PixelDataLength];
    //            colorFrame.CopyPixelDataTo(colorPixels);

    //            int colorStride = colorFrame.Width * 4;
    //            colorImage.Source = BitmapSource.Create(colorFrame.Width, colorFrame.Height,
    //                96, 96, PixelFormats.Bgr32, null, colorPixels, colorStride);
    //        }

    //        //Depth information


    //        //using (DepthImageFrame depthFrame = e.OpenDepthImageFrame())
    //        //{

    //        //    if (depthFrame == null)
    //        //    {
    //        //        return;
    //        //    }

    //        //    byte[] depthpixels = GenerateColoredBytes(depthFrame);

    //        //    int depthStride = depthFrame.Width * 4;

    //        //    depthImage.Source = BitmapSource.Create(depthFrame.Width, depthFrame.Height, 96, 96, PixelFormats.Bgr32, null, depthpixels, depthStride);

    //        //}

    //    }


    //    //private byte[] GenerateColoredBytes(DepthImageFrame depthFrame)
    //    //{

    //    //    short[] rawDepthData = new short[depthFrame.PixelDataLength];
    //    //    depthFrame.CopyPixelDataTo(rawDepthData);

    //    //    Byte[] depthColorPixels = new byte[depthFrame.Height * depthFrame.Width *4];

    //    //    const int BlueIndex = 0;
    //    //    const int GreenIndex = 1;
    //    //    const int RedIndex = 2;

    //    //    for (int depthIndex = 0, colorIndex = 0;
    //    //        depthIndex < rawDepthData.Length && colorIndex < depthColorPixels.Length;
    //    //        depthIndex++, colorIndex +=4)
    //    //    {
    //    //        int player = rawDepthData[depthIndex] & DepthImageFrame.PlayerIndexBitmask;

    //    //        int depth = rawDepthData[depthIndex] >> DepthImageFrame.PlayerIndexBitmaskWidth;

    //    //        if (depth <= 900)
    //    //        {
    //    //            depthColorPixels[colorIndex + BlueIndex] = 255;
    //    //            depthColorPixels[colorIndex + GreenIndex] = 0;
    //    //            depthColorPixels[colorIndex + RedIndex] = 0;
    //    //        }
    //    //        else if (depth > 900 && depth < 2000)
    //    //        {
    //    //            depthColorPixels[colorIndex + BlueIndex] = 0;
    //    //            depthColorPixels[colorIndex + GreenIndex] = 255;
    //    //            depthColorPixels[colorIndex + RedIndex] = 0;
    //    //        }
    //    //        else if (depth >2000)
    //    //        {
    //    //            depthColorPixels[colorIndex + BlueIndex] = 0;
    //    //            depthColorPixels[colorIndex + GreenIndex] = 0;
    //    //            depthColorPixels[colorIndex + RedIndex] = 255;
    //    //        }



    //    //    "Replaying skeleton frames"
    //    //    System.Runtime.Serialization.Formatters.Binary;
    //    //    
    //    //    public class ReplaySkeletonFrame : ReplayFrame
    //    //    {
    //    //          publicTuple<float, float, float, float>
    //    //    FloorClipPlane { get; private set; }
    //    //          public Skeleton [] Skeleton { get; private set; }
    //    //
    //    //          public ReplaySkeletonFrame(Skeleton Frame frame)
    //    //          {
    //    //              FloorClipPlane = frame.FloorClipPlane;
    //    //              FrameNumber = frame.FrameNumber;
    //    //              TimeStamp = frame.Timestamp;
    //    //              Skeletons = frame.GetSkeletons();
    //    //          }
    //    //
    //    //          public ReplaySkeletonFrame()
    //    //          {
    //    //
    //    //          }
    //    //
    //    //          internal override void
    //    //     CreateFromreader(BinaryReader reader)
    //    //          {
    //    //              TimeStamp = reader.ReadInt 64();
    //    //              FloorClipPlane = new Tuple<float, float, float, float>
    //    //                  (reader.ReadSingle(), reader.ReadSingle(),reader.ReadSingle(), reader.ReadSingle());
    //    // 
    //    //              FrameNember = reader.ReadInt32();
    //    //      
    //    //              BinaryFormatter formatter = new Binary =Formatter();
    //    //              Skeletons = (Skeleton[])formatter.Deserialize(reader.BaseStream);
    //    //          }
    //    //          public static implicit operator ReplaySkeletonFrame(SkeletonFrame frame)
    //    //          {
    //    //              return new ReplaySkeeltonFrame(frame);
    //    //          }
    //    //     }


    //    void StopKinect(KinectSensor sensor)
    //    {
    //        if (sensor != null)
    //        {
    //            sensor.Stop();
    //            sensor.AudioSource.Stop();
    //        }
    //    }

    //    private void Window_Closing(object sender, System.ComponentModel.CancelEventArgs e)
    //    {
    //        StopKinect(_sensor);
    //    }

    //}
}

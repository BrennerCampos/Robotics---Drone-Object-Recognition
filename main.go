package main

// https://pkg.go.dev/gocv.io/x/gocv@v0.25.0
// https://pkg.go.dev/gobot.io/x/gobot@v1.15.0/platforms/dji/tello

import (
	"fmt"
	"gobot.io/x/gobot"
	"gobot.io/x/gobot/platforms/dji/tello"
	"gocv.io/x/gocv"
	"image"
	"image/color"
	"io"
	"os/exec"
	"time"
)

const (
	frameSize = 960 * 720 * 3
)

var (

	// current hue values set to find red colors
	lhsv = gocv.Scalar{Val1: 0, Val2: 255, Val3: 100}
	hhsv = gocv.Scalar{Val1: 255, Val2: 255, Val3: 255}

	size = image.Point{X: 600, Y: 600}
	blur = image.Point{X: 11, Y: 11}

	window  = gocv.NewWindow("thersholded")
	window2 = gocv.NewWindow("images")
	img     = gocv.NewMat()
	mask    = gocv.NewMat()
	frame   = gocv.NewMat()
	hsv     = gocv.NewMat()
	kernel  = gocv.NewMat()
)

func main() {

	defer close()

	window.ResizeWindow(600, 600)
	window.MoveWindow(100, 0)
	window2.ResizeWindow(600, 600)
	window2.MoveWindow(800, 0)

	//video, _ := gocv.OpenVideoCapture(0)		// this will override drone camera with laptop camera

	drone := tello.NewDriver("8890")

	// open DNN object tracking model
	model := "frozen_inference_graph.pb"
	config := "ssd_mobilenet_v1_coco_2017_11_17.pbtxt"
	backend := gocv.NetBackendDefault
	target := gocv.NetTargetCPU
	net := gocv.ReadNet(model, config)
	if net.Empty() {
		fmt.Printf("Error reading network model from : %v %v\n", model, config)
		return
	}
	defer net.Close()
	net.SetPreferableBackend(gocv.NetBackendType(backend))
	net.SetPreferableTarget(gocv.NetTargetType(target))

	ffmpeg := exec.Command("ffmpeg", "-i", "pipe:0", "-pix_fmt", "bgr24", "-vcodec", "rawvideo",
		"-an", "-sn", "-s", "960x720", "-f", "rawvideo", "pipe:1")
	ffmpegIn, _ := ffmpeg.StdinPipe()
	ffmpegOut, _ := ffmpeg.StdoutPipe()

	work := func() {

		if err := ffmpeg.Start(); err != nil {
			fmt.Println(err)
			return
		}

		drone.On(tello.ConnectedEvent, func(data interface{}) {
			fmt.Println("Successful connection.")
			drone.StartVideo()
			drone.SetExposure(1)
			drone.SetVideoEncoderRate(4)

			gobot.Every(100*time.Millisecond, func() {
				drone.StartVideo()
			})
		})

		drone.On(tello.VideoFrameEvent, func(data interface{}) {
			pkt := data.([]byte)
			if _, err := ffmpegIn.Write(pkt); err != nil {
				fmt.Println(err)
			}
		})
	}

	robot := gobot.NewRobot("tello",
		[]gobot.Connection{},
		[]gobot.Device{drone},
		work,
	)

	robot.Start(false)

	err := drone.TakeOff()
	if err != nil {
		fmt.Println(err)
	}

	for {
		buf := make([]byte, frameSize)
		if _, err := io.ReadFull(ffmpegOut, buf); err != nil {
			fmt.Println(err)
			continue
		}

		img, _ = gocv.NewMatFromBytes(720, 960, gocv.MatTypeCV8UC3, buf)

		// for use of webcam
		//if !video.Read(&img){
		//	break
		//}

		if img.Empty() {
			continue
		}

		// filters
		gocv.Flip(img, &img, 1)
		gocv.Resize(img, &img, size, 0, 0, gocv.InterpolationLinear)
		gocv.GaussianBlur(img, &frame, blur, 0, 0, gocv.BorderReflect101) // blurr img
		gocv.CvtColor(frame, &hsv, gocv.ColorBGRToHSV)                    // convert blurr to HSV color

		// look for object
		gocv.InRangeWithScalar(hsv, lhsv, hhsv, &mask) // find pixels based on hhsv and lhsv
		gocv.Erode(mask, &mask, kernel)
		gocv.Dilate(mask, &mask, kernel)

		contour := bestContour(mask, 2000)
		if len(contour) == 0 { // if no contour present; loop will continue

			if winShow() {
				break
			}
			continue
		}

		rect := gocv.BoundingRect(contour)
		gocv.Rectangle(&img, rect, color.RGBA{0, 255, 0, 0}, 2)

		droneRect := image.Point{X: 300, Y: 300}
		objRect := image.Point{X: (rect.Max.X + rect.Min.X) / 2, Y: (rect.Max.Y + rect.Min.Y) / 2}
		diffRect := image.Point{X: droneRect.X - objRect.X, Y: droneRect.Y - objRect.Y}

		currentArea := rect.Dx() * rect.Dy()

		s1 := 30
		s2 := 20
		s3 := 15
		s4 := 7
		s5 := 2

		// note: teh ranges of x and y coordinates range between -300 to 300 therefore s1 and s2 will never be reached
		// X direction
		if diffRect.X <= -1 { // left side

			drone.Right(0)

			if diffRect.X <= -500 {
				drone.Left(s1)
			} else if diffRect.X <= -400 && diffRect.X > -500 {
				drone.Left(s2)
			} else if diffRect.X <= -200 && diffRect.X > -400 {
				drone.Left(s3)
			} else if diffRect.X <= -50 && diffRect.X > -200 {
				drone.Left(s4)
			} else if diffRect.X < -1 && diffRect.X > -50 {
				drone.Left(s5)
			} else if diffRect.X == -1 {
				drone.Left(0)
			}

		} else if diffRect.X >= 1 { // right side

			drone.Left(0)

			if diffRect.X >= 500 {
				drone.Right(s1)
			} else if diffRect.X >= 400 && diffRect.X < 500 {
				drone.Right(s2)
			} else if diffRect.X >= 200 && diffRect.X < 400 {
				drone.Right(s3)
			} else if diffRect.X >= 50 && diffRect.X < 200 {
				drone.Right(s4)
			} else if diffRect.X > 1 && diffRect.X < 50 {
				drone.Right(s5)
			} else if diffRect.X == 1 {
				drone.Right(0)
			}

		}

		// Y direction
		if diffRect.Y <= -1 { // bottom half

			drone.Up(0)

			if diffRect.Y <= -500 {
				drone.Down(s1 * 2)
			} else if diffRect.Y <= -400 && diffRect.Y > -500 {
				drone.Down(s2 * 2)
			} else if diffRect.Y <= -200 && diffRect.Y > -400 {
				drone.Down(s3 * 2)
			} else if diffRect.Y <= -50 && diffRect.Y > -200 {
				drone.Down(s4 * 2)
			} else if diffRect.Y < -1 && diffRect.Y > -50 {
				drone.Down(s5)
			} else if diffRect.Y == -1 {
				drone.Down(0)
			}

		} else if diffRect.Y >= 1 { // top half
			drone.Down(0)

			if diffRect.Y >= 500 {
				drone.Up(s1)
			} else if diffRect.Y >= 400 && diffRect.Y < 500 {
				drone.Up(s2)
			} else if diffRect.Y >= 200 && diffRect.Y < 400 {
				drone.Up(s3)
			} else if diffRect.Y >= 50 && diffRect.Y < 200 {
				drone.Up(s4)
			} else if diffRect.Y >= 1 && diffRect.Y < 50 {
				drone.Up(s5)
			} else if diffRect.Y == 1 {
				drone.Up(0)
			}

		}

		// z direction
		sz := 10

		if currentArea >= 0 && currentArea < 2500 { // between 0 & 100x100 area - do nothing
			fmt.Println("where did the box go? hover.")
			drone.Hover()

		} else if currentArea >= 2500 && currentArea < 30000 { //  between 100x100 & 200x200
			fmt.Println("too far, moving forward")
			drone.Forward(sz)

		} else if currentArea >= 50000 && currentArea < 360000 { // between 600x600 & 300x300 area
			fmt.Println("too close, moving backward")
			drone.Backward(sz)

		} else if currentArea >= 30000 && currentArea <= 50000 { // between 200x200 & 300x300
			fmt.Println("and now just right")
			drone.Forward(0)
			drone.Backward(0)
		}

		// safety turn off - put object in main area of drone
		if currentArea > 160000 || currentArea > 160000 && currentArea == 0 { // ~300x300 area
			drone.Land()
		}

		if winShow() {
			break
		}

	} // end of for-ever
} // end of main

func bestContour(frame gocv.Mat, minArea float64) []image.Point {
	cntr := gocv.FindContours(frame, gocv.RetrievalExternal, gocv.ChainApproxSimple)
	var (
		bestCntr []image.Point
		bestArea = minArea
	)
	for _, cnt := range cntr {
		if area := gocv.ContourArea(cnt); area > bestArea {
			bestArea = area
			bestCntr = cnt
		}
	}
	return bestCntr
}

func winShow() bool {
	window2.IMShow(img) // wi
	window.IMShow(mask)
	return window2.WaitKey(1) == 27 || window.WaitKey(1) == 27
}

func close() {
	defer window2.Close()
	defer window.Close()
	defer img.Close()
	defer mask.Close()
	defer frame.Close()
	defer hsv.Close()
	defer kernel.Close()
}

#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <pthread.h>
#include <string>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/ximgproc/disparity_filter.hpp>
#include "StereoVisionBasedDisparityMap.h"
#include "ObstacleDetectionAlgorithm.h"
#include "mraa.hpp"

using namespace std;
using namespace cv;
using namespace cv::ximgproc;



int LeftMotorpin = 26;
int MiddleMotorpin = 24;
int RightMotorpin = 22;
int interruptPin1 = 18;
int interruptPin2 = 20;
int leftcameraport = 0;
int rightcameraport = 1;

int running = 0;
static volatile float marks ;
static volatile float range ;
static volatile int mode = 1;
static volatile bool changeofstate = 0;
vector<float> stereoVisionBasedObstacleDetection(Mat left_camera_frame, Mat right_camera_frame, Mat scene);

struct thread_data{
   mraa::Gpio* gpio;
   float duty;
};

void sig_handler(int signo)
{
    if (signo == SIGINT) {
        printf("closing IO nicely\n");
        running = -1;
    }
}
void softPwmWrite (float value)
{
    if (value < 0)
      value = 0 ;
    else if (value > 1.0)
      value =  1.0;
    marks  = value ;
}

void *softPwmThread(void *arg)
{
  struct thread_data *my_data;
  my_data = (struct thread_data *) arg;
  float mark, space ;
  float range = 1.0f;
  marks = my_data->duty;
  struct sched_param param ;
  param.sched_priority = sched_get_priority_max (SCHED_RR) ;
  pthread_setschedparam (pthread_self (), SCHED_RR, &param) ;
  for (;;)
  {
    mark  = marks ;
    space = range  - mark ;

    if (mark != 0)
    {
      my_data->gpio->write(1) ;
      usleep (mark * 600.0) ;
    }

    if (space != 0)
    {
      my_data->gpio->write(0);
      usleep (space * 600.0) ;
    }
  }

  return NULL ;
}

void *VideoStreamingThread(void *arg)
{
	struct sched_param param ;
        param.sched_priority = sched_get_priority_max (SCHED_RR) ;
        pthread_setschedparam (pthread_self (), SCHED_RR, &param) ;
	char cmd[] ="-s 320x240 -f video4linux2 -i /dev/video0 -f mpeg1video -b 800k -r 30 https://simplenodechat-error005.c9users.io:8082";
	system( "mosquitto_pub -h iot.eclipse.org -t 'artificialeyes/message' -m 'calling'");
	usleep(3000);
	system( "mosquitto_pub -h iot.eclipse.org -t 'artificialeyes/phone' -m 'call'");
	//execl("/usr/bin/ffmpeg","ffmpeg","-s", "320x240", "-f", "video4linux2", "-i", "/dev/video0", "-f", "mpeg1video", "-b", "800k", "-r", "30", "https://simplenodechat-error005.c9users.io:8082",(char*) NULL);
	system("ffmpeg -s 320x240 -f video4linux2 -i /dev/video0 -f mpeg1video -b 800k -r 30 https://simplenodechat-error005.c9users.io:8082");
	//pthread_exit(NULL);
}
void interrupt(void* args) //interrupt to take image and upload
{
   if (mode == 1)
	mode = 2;
}

void interrupt2(void* args) //interrupt to perform video streaming
{
   pthread_t *thisthread;
   thisthread = (pthread_t *)args;
   if (changeofstate == 0)
   {
	   mode = 3;
	   pthread_create (thisthread, NULL, VideoStreamingThread, NULL) ;
	   changeofstate = !changeofstate;
   }
   else
   {
	   mode = 1;
	  
	   //pthread_kill(*thisthread,SIGINT);
	  
	   pthread_cancel(*thisthread);
	   pthread_join(*thisthread,NULL);
	   system( "mosquitto_pub -h iot.eclipse.org -t 'artificialeyes/phone' -m 'end'");
	   changeofstate = !changeofstate;
   }
}

int main(int argc, char** argv)
{
    signal(SIGINT, sig_handler);
	
	// pin initialization
	mraa::Pwm* pwmR = new mraa::Pwm(RightMotorpin);
    mraa::Pwm* pwmM = new mraa::Pwm(MiddleMotorpin);
    mraa::Gpio* pwmL = new mraa::Gpio(LeftMotorpin);
    mraa::Gpio* isr1 = new mraa::Gpio(interruptPin1);
	mraa::Gpio* isr2 = new mraa::Gpio(interruptPin2);
    pthread_t myThread ,streamingThread;
    isr1->dir(mraa::DIR_IN);
    isr1->isr(mraa::EDGE_RISING, &interrupt, NULL);
    isr2->dir(mraa::DIR_IN);
    isr2->isr(mraa::EDGE_RISING, &interrupt2, (void *)&streamingThread);
	
    pwmR->enable(true);
	pwmM->enable(true);
    pwmL->dir(mraa::DIR_OUT);
	
    struct thread_data td;
    td.gpio= pwmL;
    td.duty=0.0;
    pthread_create (&myThread, NULL, softPwmThread, (void *)&td) ;
    pwmL->write(0);
    //pid_t PID1;
	VideoCapture capL(leftcameraport); // open the left camera
	VideoCapture capR(rightcameraport); // open the right camera
	capL.set(CV_CAP_PROP_FPS,15);
	capR.set(CV_CAP_PROP_FPS,15);
	//capL.set(CV_CAP_PROP_BUFFERSIZE, 3);
	//capR.set(CV_CAP_PROP_BUFFERSIZE, 3);
    //softPwmWrite(value);
    while (running == 0) {
		if (mode == 1)
		{
			/*if (&streamingThread !=NULL)
			{
				cout<<"end streamingThread\n";
				system( "mosquitto_pub -h iot.eclipse.org -t 'artificialeyes/phone' -m 'end'");
				pthread_cancel(streamingThread);
				pthread_join(streamingThread,NULL);
			}	*/
			/*while (!pthread_equal(streamingThread, pthread_self()))
			{
				cout<<"ending process"<<endl;
				pthread_cancel(streamingThread);
                                pthread_join(streamingThread,NULL);
			}*/
			if (!capL.isOpened() ||!capR.isOpened())
			{
				//kill(PID1,15);
				//pthread_cancel(streamingThread);
				//pthread_join(streamingThread,NULL);
			    if (!capL.isOpened())
				capL.open(leftcameraport);
			    if (!capR.isOpened())
				capR.open(rightcameraport);
			}
			//grab frame
			Mat LeftFrame,RightFrame,LeftFrameG,RightFrameG;
			//double total_time;
			//total_time = (double)getTickCount();
			capL.grab();
			capR.grab();
			capL.grab();
                        capR.grab();
			capL.grab();
                        capR.grab();
			capL.grab();
                        capR.grab();
			capL.grab();
                        capR.grab();
			capL.grab();
                        capR.grab();
			capL.grab();
                        capR.grab();
			//total_time = ((double)getTickCount()-total_time)/getTickFrequency();
			//cout.precision(5);
			//cout<<"Total time: "<<total_time<<endl;
			capL.retrieve(LeftFrame);
			capR.retrieve(RightFrame);
			cvtColor( LeftFrame, LeftFrameG, CV_BGR2GRAY );
			cvtColor( RightFrame, RightFrameG, CV_BGR2GRAY );
			vector<float> message = stereoVisionBasedObstacleDetection(LeftFrameG, RightFrameG, LeftFrame);
			//total_time = ((double)getTickCount() - total_time) / getTickFrequency();
			//cout.precision(5);
			//cout << "Total time:  " << total_time << "s" << endl;
			softPwmWrite(message[0]);
			pwmM->write(message[1]);
			pwmR->write(message[2]);
			cout.precision(3);
			cout<<"\n" <<message[0]<< "            " <<message[1]<< "          " << message[2]<<endl;
			
		}
		else if (mode == 2)
		{
			cout<< "mode 2\n";
			system( "mosquitto_pub -h iot.eclipse.org -t 'artificialeyes/message' -m 'uploading'");
			if (capL.isOpened() || capR.isOpened())
			{
				cout<<"close camera mode 2\n";
				capL.release();
				capR.release();
			}
			system("./uploadimage.sh");
			mode = 1;
		}
		else if (mode == 3)
		{
			softPwmWrite(0.0);
			pwmM->write(0.0);
			pwmR->write(0.0);
			
			if (capL.isOpened() || capR.isOpened())
			{
				cout<<"close camera mode 3\n";
				capL.release();
				capR.release();
			}
		}
    }
	
    pthread_cancel(myThread);
    pthread_join(myThread,NULL);
    pwmL->write(0);
    //softPwmWrite(0.0);
    pwmM->write(0.0);
    pwmR->write(0.0);
    delete pwmR,pwmL,pwmM,isr1,isr2;
    cout<<"terminate GPIO\n";
    return MRAA_SUCCESS;
}

vector<float> stereoVisionBasedObstacleDetection(Mat left_camera_frame, Mat right_camera_frame,Mat scene)
{
	// ===== Read stereo calibration file ===== //
	string calib_filename = "9_stereo_calib.yml";
	Mat K_left, K_right;
	Mat R_left, R_right;
	Mat P_left, P_right;
	Mat D_left, D_right;
	Mat Q;
	readStereoCalibrationFile(calib_filename, K_left, K_right, D_left, D_right, R_left, R_right, P_left, P_right, Q);

	// ===== Rectify stereo image pair ===== //
	//Mat scene = left_img.clone();
	//Mat left_camera_frame = imread(left_img_filename, CV_LOAD_IMAGE_GRAYSCALE);
	//Mat right_camera_frame = imread(right_img_filename, CV_LOAD_IMAGE_GRAYSCALE);
	stereoRectifytImage(left_camera_frame, right_camera_frame, K_left, K_right, D_left, D_right, R_left, R_right, P_left, P_right);

	// ===== Generate disparity map ===== //
	Mat disparity_map;
	generateDepthMapSGBM(left_camera_frame, right_camera_frame, disparity_map, 5.0, 10000.0, 2.0, 13, (16 * 1));

	// ===== Detect Obstacle ===== //
	Mat process_disp_img = disparity_map.clone();
	Mat u_disp = generateDisparityMapU(disparity_map, 255);
	eliminateGroundPlane(process_disp_img, u_disp, 48, 20);
	postProcessingDisparityMap(process_disp_img);
	vector<float> message = disparityToDistance(process_disp_img, disparity_map, scene, Q);

	//imshow("Left frame", left_camera_frame);
	//imshow("Right frame", right_camera_frame);
	imshow("Disparity map", disparity_map);
	//imshow("Processed Disparity", process_disp_img);
	//imshow("Scene", scene);
	waitKey(1);

	return message;
}

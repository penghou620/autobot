// openni_tracker.cpp

#include <ros/ros.h>
#include <ros/package.h>
#include <tf/transform_broadcaster.h>
#include <kdl/frames.hpp>
#include <geometry_msgs/Point.h>
#include <std_msgs/String.h>

#include <XnOpenNI.h>
#include <XnCodecIDs.h>
#include <XnCppWrapper.h>
#include <math.h>

using std::string;


xn::Context        g_Context;
xn::DepthGenerator g_DepthGenerator;
xn::UserGenerator  g_UserGenerator;

XnBool g_bNeedPose   = FALSE;
XnChar g_strPose[20] = "";

ros::Publisher head_pub;
ros::Publisher gesture_pub;
void XN_CALLBACK_TYPE User_NewUser(xn::UserGenerator& generator, XnUserID nId, void* pCookie) {
	ROS_INFO("New User %d", nId);

	if (g_bNeedPose)
		g_UserGenerator.GetPoseDetectionCap().StartPoseDetection(g_strPose, nId);
	else
		g_UserGenerator.GetSkeletonCap().RequestCalibration(nId, TRUE);
}

void XN_CALLBACK_TYPE User_LostUser(xn::UserGenerator& generator, XnUserID nId, void* pCookie) {
	ROS_INFO("Lost user %d", nId);
}

void XN_CALLBACK_TYPE UserCalibration_CalibrationStart(xn::SkeletonCapability& capability, XnUserID nId, void* pCookie) {
	ROS_INFO("Calibration started for user %d", nId);
}

void XN_CALLBACK_TYPE UserCalibration_CalibrationEnd(xn::SkeletonCapability& capability, XnUserID nId, XnBool bSuccess, void* pCookie) {
	if (bSuccess) {
		ROS_INFO("Calibration complete, start tracking user %d", nId);
		g_UserGenerator.GetSkeletonCap().StartTracking(nId);
	}
	else {
		ROS_INFO("Calibration failed for user %d", nId);
		if (g_bNeedPose)
			g_UserGenerator.GetPoseDetectionCap().StartPoseDetection(g_strPose, nId);
		else
			g_UserGenerator.GetSkeletonCap().RequestCalibration(nId, TRUE);
	}
}

void XN_CALLBACK_TYPE UserPose_PoseDetected(xn::PoseDetectionCapability& capability, XnChar const* strPose, XnUserID nId, void* pCookie) {
    ROS_INFO("Pose %s detected for user %d", strPose, nId);
    g_UserGenerator.GetPoseDetectionCap().StopPoseDetection(nId);
    g_UserGenerator.GetSkeletonCap().RequestCalibration(nId, TRUE);
}

void publishTransform(XnUserID const& user, XnSkeletonJoint const& joint, string const& frame_id, string const& child_frame_id) {
    static tf::TransformBroadcaster br;

    XnSkeletonJointPosition joint_position;
    g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(user, joint, joint_position);
    double x = -joint_position.position.X / 1000.0;
    double y = joint_position.position.Y / 1000.0;
    double z = joint_position.position.Z / 1000.0;

    XnSkeletonJointOrientation joint_orientation;
    g_UserGenerator.GetSkeletonCap().GetSkeletonJointOrientation(user, joint, joint_orientation);

    XnFloat* m = joint_orientation.orientation.elements;
    KDL::Rotation rotation(m[0], m[1], m[2], m[3], m[4], m[5], m[6], m[7], m[8]);
    double qx, qy, qz, qw;
    rotation.GetQuaternion(qx, qy, qz, qw);

    char child_frame_no[128];
    snprintf(child_frame_no, sizeof(child_frame_no), "%s_%d", child_frame_id.c_str(), user);

    tf::Transform transform;
    transform.setOrigin(tf::Vector3(x, y, z));
    transform.setRotation(tf::Quaternion(qx, -qy, -qz, qw));

    // #4994
    tf::Transform change_frame;
    change_frame.setOrigin(tf::Vector3(0, 0, 0));
    tf::Quaternion frame_rotation;
    frame_rotation.setEulerZYX(1.5708, 0, 1.5708);
    change_frame.setRotation(frame_rotation);

    transform = change_frame * transform;

    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), frame_id, child_frame_no));
}

void publishTransforms(const std::string& frame_id) {
    XnUserID users[15];
    XnUInt16 users_count = 15;
    g_UserGenerator.GetUsers(users, users_count);

    for (int i = 0; i < users_count; ++i) {
        XnUserID user = users[i];
        if (!g_UserGenerator.GetSkeletonCap().IsTracking(user))
            continue;


        publishTransform(user, XN_SKEL_HEAD,           frame_id, "head");
        publishTransform(user, XN_SKEL_NECK,           frame_id, "neck");
        publishTransform(user, XN_SKEL_TORSO,          frame_id, "torso");

        publishTransform(user, XN_SKEL_LEFT_SHOULDER,  frame_id, "left_shoulder");
        publishTransform(user, XN_SKEL_LEFT_ELBOW,     frame_id, "left_elbow");
        publishTransform(user, XN_SKEL_LEFT_HAND,      frame_id, "left_hand");

        publishTransform(user, XN_SKEL_RIGHT_SHOULDER, frame_id, "right_shoulder");
        publishTransform(user, XN_SKEL_RIGHT_ELBOW,    frame_id, "right_elbow");
        publishTransform(user, XN_SKEL_RIGHT_HAND,     frame_id, "right_hand");

        publishTransform(user, XN_SKEL_LEFT_HIP,       frame_id, "left_hip");
        publishTransform(user, XN_SKEL_LEFT_KNEE,      frame_id, "left_knee");
        publishTransform(user, XN_SKEL_LEFT_FOOT,      frame_id, "left_foot");

        publishTransform(user, XN_SKEL_RIGHT_HIP,      frame_id, "right_hip");
        publishTransform(user, XN_SKEL_RIGHT_KNEE,     frame_id, "right_knee");
        publishTransform(user, XN_SKEL_RIGHT_FOOT,     frame_id, "right_foot");
    }
}

#define CHECK_RC(nRetVal, what)										\
	if (nRetVal != XN_STATUS_OK)									\
	{																\
		ROS_ERROR("%s failed: %s", what, xnGetStatusString(nRetVal));\
		return nRetVal;												\
	}

int CheckPose(XnUserID nId){
	int NOT_IN_POSE = 0;
	int IN_POSE_FOR_LITTLE_TIME = 1;
    XnSkeletonJointPosition leftHand;
    XnSkeletonJointPosition rightHand;
    XnSkeletonJointPosition leftElbow;
    XnSkeletonJointPosition rightElbow;
	XnSkeletonJointPosition head;
    xn::SkeletonCapability skeletonCap = g_UserGenerator.GetSkeletonCap();

    if (!skeletonCap.IsTracking(nId))
    {
            return NOT_IN_POSE;
    }

    skeletonCap.GetSkeletonJointPosition(nId,XN_SKEL_LEFT_HAND, leftHand);
    skeletonCap.GetSkeletonJointPosition(nId,XN_SKEL_RIGHT_HAND, rightHand);
    skeletonCap.GetSkeletonJointPosition(nId,XN_SKEL_LEFT_ELBOW, leftElbow);
    skeletonCap.GetSkeletonJointPosition(nId,XN_SKEL_RIGHT_ELBOW, rightElbow);

	float xDist = rightHand.position.X - rightElbow.position.X;
	float yDist = rightHand.position.Y - rightElbow.position.Y;
	float angle = atan2(yDist, xDist);	
	printf("angle:%f\n",angle);
	//for some reason, the lefthand store the data of my right hand
	//printf("Left Hand X:%f\n",leftHand.position.X);
	//printf("Left Hand Y:%f\n",leftHand.position.Y);
	//printf("Left Hand Z:%f\n",leftHand.position.Z);
	//printf("Left Hand X:%f\n",leftElbow.position.X);
	//printf("Left Hand Y:%f\n",leftElbow.position.Y);
	//printf("Left Hand Z:%f\n",leftElbow.position.Z);

	std_msgs::String gesture_result;	
	if(fabs(angle) < 0.08){
		gesture_result.data = "start";
	}else if(angle < 1.66 || angle > 1.48 ){//1.66rad = 95deg;1.48rad=85deg
		gesture_result.data = "stop";
	}
	gesture_pub.publish(gesture_result);

	skeletonCap.GetSkeletonJointPosition(nId,XN_SKEL_HEAD,head);
	geometry_msgs::Point head_coordinates;
	head_coordinates.x = head.position.X;
	head_coordinates.y = head.position.Y;
	head_coordinates.z = head.position.Z;
	head_pub.publish(head_coordinates);

    // bool bHaveLeftHand = leftHand.position.fConfidence  >= 0.5;
    // bool bHaveRightHand = rightHand.position.fConfidence >= 0.5;
    // if(!bHaveLeftHand && !bHaveRightHand )
    // {
    //         return NOT_IN_POSE;
    // }
    // if(bHaveLeftHand) m_prevLeftHand  = leftHand;
    // if(bHaveRightHand) m_prevRightHand = rightHand;
    //check for X (left hand is "righter" than right (more than 10 cm)
    //float xDist = leftHand.position.position.X - rightHand.position.position.X ;
    //if(xDist < 100 ) return NOT_IN_POSE;
    //check hands to be at same height
    //float yDist = fabs(leftHand.position.position.Y - rightHand.position.position.Y);
    //if(yDist > 300 ) return NOT_IN_POSE;

    return IN_POSE_FOR_LITTLE_TIME; 
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "openni_tracker");
	ros::NodeHandle nh;
	head_pub = nh.advertise<geometry_msgs::Point>("head",1000);
	gesture_pub = nh.advertise<std_msgs::String>("gesture",1000);
    string configFilename = ros::package::getPath("openni_tracker") + "/openni_tracker.xml";
    XnStatus nRetVal = g_Context.InitFromXmlFile(configFilename.c_str());
    CHECK_RC(nRetVal, "InitFromXml");

    nRetVal = g_Context.FindExistingNode(XN_NODE_TYPE_DEPTH, g_DepthGenerator);
    CHECK_RC(nRetVal, "Find depth generator");

	nRetVal = g_Context.FindExistingNode(XN_NODE_TYPE_USER, g_UserGenerator);
	if (nRetVal != XN_STATUS_OK) {
		nRetVal = g_UserGenerator.Create(g_Context);
		CHECK_RC(nRetVal, "Find user generator");
	}

	if (!g_UserGenerator.IsCapabilitySupported(XN_CAPABILITY_SKELETON)) {
		ROS_INFO("Supplied user generator doesn't support skeleton");
		return 1;
	}

    XnCallbackHandle hUserCallbacks;
	g_UserGenerator.RegisterUserCallbacks(User_NewUser, User_LostUser, NULL, hUserCallbacks);

	XnCallbackHandle hCalibrationCallbacks;
	g_UserGenerator.GetSkeletonCap().RegisterCalibrationCallbacks(UserCalibration_CalibrationStart, UserCalibration_CalibrationEnd, NULL, hCalibrationCallbacks);

	if (g_UserGenerator.GetSkeletonCap().NeedPoseForCalibration()) {
		g_bNeedPose = TRUE;
		if (!g_UserGenerator.IsCapabilitySupported(XN_CAPABILITY_POSE_DETECTION)) {
			ROS_INFO("Pose required, but not supported");
			return 1;
		}

		XnCallbackHandle hPoseCallbacks;
		g_UserGenerator.GetPoseDetectionCap().RegisterToPoseCallbacks(UserPose_PoseDetected, NULL, NULL, hPoseCallbacks);

		g_UserGenerator.GetSkeletonCap().GetCalibrationPose(g_strPose);
	}

	g_UserGenerator.GetSkeletonCap().SetSkeletonProfile(XN_SKEL_PROFILE_ALL);

	nRetVal = g_Context.StartGeneratingAll();
	CHECK_RC(nRetVal, "StartGenerating");

	ros::Rate r(30);

    ros::NodeHandle pnh("~");
    string frame_id("openni_depth_frame");
    pnh.getParam("camera_frame_id", frame_id);
                
	while (ros::ok()) {
		g_Context.WaitAndUpdateAll();
		publishTransforms(frame_id);
		XnUserID aUsers[15];
		XnUInt16 nUsers = 15;
		g_UserGenerator.GetUsers(aUsers, nUsers);
		for(int i = 0; i < nUsers; ++i){
			if(g_UserGenerator.GetSkeletonCap().IsTracking(aUsers[i])){
				if(CheckPose(aUsers[i]) == 1){
                    //printf("X Pose Detected");
                }
			}
		}
		r.sleep();
	}

	g_Context.Shutdown();
	return 0;
}

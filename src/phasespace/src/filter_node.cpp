#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <phasespace/FullState.h>

#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Geometry>

// need a class in order publish in the callback
class SubscribeAndPublish
{
  ros::NodeHandle nh;
  ros::Publisher velPub;
  ros::Publisher bodyVelPub;
  ros::Publisher odomPub;
  ros::Publisher fullPub;
  ros::Subscriber mocapSub;
  
  int window_size;            // Number of data points to store
  int polyOrder;              // Order of best fit polynomial
  bool buffer_full;           // Estimation will start after buffer is full for first time
  Eigen::VectorXd t_buff;            // ring buffer for time data
  Eigen::MatrixXd pos_buff;          // ring buffer for position data
  Eigen::MatrixXd quat_buff;         // ring buffer for orientation data
  int ind;                    // index of oldest data in buffers. Data at this index gets replaced with new data
  int tInd;                   // index of oldest time data. Data at this index gets replaced with new data
  int pInd;                   // index of oldest position data Data at this index gets replaced with new data
  int qInd;                   // index of oldest orientation data Data at this index gets replaced with new data
  std::string this_frame;     // frame name of the rigid body
public:
  SubscribeAndPublish()
  {
    // Parameters
    ros::NodeHandle nhp("~");
    nhp.param<int>("window_size",window_size,30);
    nhp.param<int>("polyOrder",polyOrder,2);
    this_frame = ros::this_node::getNamespace().substr(ros::this_node::getNamespace().find_first_not_of("/"));

    // Initialize buffers
    ind = 0;
    t_buff.resize(window_size);
    pos_buff.resize(window_size,3);
    quat_buff.resize(window_size,4);
    buffer_full = false;
    
    // Publishers
    velPub = nh.advertise<geometry_msgs::TwistStamped>("vel",1);
    bodyVelPub = nh.advertise<geometry_msgs::TwistStamped>("body_vel",1);
    odomPub = nh.advertise<nav_msgs::Odometry>("odom",1);
    fullPub = nh.advertise<phasespace::FullState>("full_state",1);

    //Mocap subscriber
    mocapSub = nh.subscribe("pose",10,&SubscribeAndPublish::poseCB,this);
  }
  
  void poseCB(const geometry_msgs::PoseStampedConstPtr& pose)
  {
    // Setting up least squares problem A*theta = P. theta is made up of the coefficients for the best fit polynomial,
    // Velocity/Acceleration is estimated using analytical derivatives. 
    // Each block of data is arranged like this:
    // [X0, Y0, Z0]     [1,T0,T0^2] * [p0x, p0y, p0z]
    // [X1, Y1, Z1]     [1,T1,T1^2]   [p1x, p1y, p1z]
    //       :                :              :
    // [Xn, Yn, Zn]     [1,Tn,Tn^2]   [pmx, pmy, pmz]
    //  \___  ___/       \___  ___/    \____  _____/
    //      \/               \/             \/
    //      Di               Ai            theta

    // Fill buffers
    t_buff(ind) = pose->header.stamp.toSec();
    pos_buff.row(ind) << pose->pose.position.x, pose->pose.position.y, pose->pose.position.z;
    Eigen::Vector4d q;
    q << pose->pose.orientation.x, pose->pose.orientation.y, pose->pose.orientation.z, pose->pose.orientation.w;
    int lastInd = (ind + (window_size-1))%(window_size); // decrement with rollover. Can't do (ind - 1)%(window_size), it results in negative number
    Eigen::Vector4d lastQ = quat_buff.row(lastInd).transpose();
    if ((lastQ-(-1*q)).norm() < (lastQ-q).norm()) { q *= -1; } // deal with equivalent quaternions
    quat_buff.row(ind) << q.transpose();
    
    // Increment index, roll back over
    ind  = (ind+1)%window_size;

    // If the index has rolled over once, the buffer is full
    if (ind == 0) { buffer_full = true; }

    if (buffer_full)
    {
      // normalize time for numerical stability/accuracy of subsequent matrix inversion
      double delT = t_buff.maxCoeff() - t_buff.minCoeff();
      Eigen::VectorXd t_norm = (t_buff.array() - t_buff.minCoeff())/delT;

      // Solve LLS for best fit line parameters
      Eigen::MatrixXd A(window_size,polyOrder+1);
      A.col(0) << Eigen::VectorXd::Ones(window_size);
      A.col(1) << t_norm;
      for (int i = 2; i < polyOrder+1; i++) { A.col(i) << (t_norm.array().pow(i)).matrix(); }
      Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeThinU | Eigen::ComputeThinV);
      svd.compute(A);
      Eigen::MatrixXd theta_pos = svd.solve(pos_buff);
      Eigen::MatrixXd theta_quat = svd.solve(quat_buff);

      // Get velocities (linear in world coordinates, angular in body coordinates)
      Eigen::MatrixXd pv = polyDerivCoeff(theta_pos, 1);
      Eigen::MatrixXd pa = polyDerivCoeff(theta_pos, 2);
      Eigen::MatrixXd pq = polyDerivCoeff(theta_quat, 1);
      Eigen::Vector3d v = polyVal(pv, 1.0).transpose()/delT;
      Eigen::Vector3d a = polyVal(pa, 1.0).transpose()/std::pow(delT,2);
      Eigen::Vector4d qDot = polyVal(pq, 1.0).transpose()/delT;
      Eigen::Matrix<double,4,3> B;
      diffMat(q,B);
      Eigen::Vector3d wbody = 2*(B.transpose())*qDot;

      // Transform velocities (linear in body coordinates, angular in world coordinates)
      Eigen::Quaterniond quat(q(3),q(0),q(1),q(2));
      Eigen::Vector3d w = quat*wbody;
      Eigen::Vector3d vbody = quat.conjugate()*v;
      //Vector3d abody = quat.conjugate()*a;

      // Publish
      geometry_msgs::TwistStamped msg = geometry_msgs::TwistStamped();
      msg.header.stamp = pose->header.stamp;
      msg.header.frame_id = pose->header.frame_id;
      msg.twist.linear.x = v(0);
      msg.twist.linear.y = v(1);
      msg.twist.linear.z = v(2);
      msg.twist.angular.x = w(0);
      msg.twist.angular.y = w(1);
      msg.twist.angular.z = w(2);
      velPub.publish(msg);

      geometry_msgs::TwistStamped msgbody = geometry_msgs::TwistStamped();
      msgbody.header.stamp = pose->header.stamp;
      msgbody.header.frame_id = this_frame; // pose->header.frame_id+"_body";
      msgbody.twist.linear.x = vbody(0);
      msgbody.twist.linear.y = vbody(1);
      msgbody.twist.linear.z = vbody(2);
      msgbody.twist.angular.x = wbody(0);
      msgbody.twist.angular.y = wbody(1);
      msgbody.twist.angular.z = wbody(2);
      bodyVelPub.publish(msgbody);

      nav_msgs::Odometry odomMsg;
      odomMsg.header.stamp = pose->header.stamp;
      odomMsg.header.frame_id = pose->header.frame_id; //"world";
      odomMsg.child_frame_id = pose->header.frame_id;
      odomMsg.pose.pose = pose->pose;
      odomMsg.twist.twist = msg.twist;
      odomPub.publish(odomMsg);

      phasespace::FullState stateMsg;
      stateMsg.header.stamp = pose->header.stamp;
      stateMsg.header.frame_id = pose->header.frame_id; //"world";
      stateMsg.pose = pose->pose;
      stateMsg.twist = msg.twist;
      stateMsg.accel.linear.x = a(0);
      stateMsg.accel.linear.y = a(1);
      stateMsg.accel.linear.z = a(2);
      fullPub.publish(stateMsg);
    }
  }
      
  // Calculate differential matrix for relationship between quaternion derivative and angular velocity.
  // qDot = 1/2*B*omega => omega = 2*B^T*qDot 
  // See strapdown inertial book. If quaternion is orientation of frame 
  // B w.r.t N in the sense that nP = q*bP*q', omega is ang. vel of frame B w.r.t. N,
  // i.e. N_w_B, expressed in the B coordinate system
  void diffMat(const Eigen::Vector4d &q, Eigen::Matrix<double,4,3> &B)
  {
    B << q(3), -q(2), q(1), q(2), q(3), -q(0), -q(1), q(0), q(3), -q(0), -q(1), -q(2);
  }

  int factorial(int x) { return ((x == 0) || (x == 1)) ? 1 : x*factorial(x-1); }

  Eigen::VectorXd polyDerivFactor(int polyOrder, int r)
  {
    Eigen::VectorXd factor = Eigen::VectorXd::Zero(polyOrder+1);
    for (int i = 0; i < polyOrder+1-r; i++)
    {
      factor(i+r) = (double) factorial(r+i)/factorial(i);
    }
    return factor;
  }
  
  Eigen::MatrixXd polyDerivCoeff(Eigen::MatrixXd pIn, int r)
  {
    int polyOrder = pIn.rows()-1;
    Eigen::VectorXd factor = polyDerivFactor(polyOrder,r);
    return pIn.bottomRows(polyOrder+1-r).array().colwise()*factor.tail(polyOrder+1-r).array();
  }

  Eigen::MatrixXd polyVal(Eigen::MatrixXd p, double t)
  {
    return (p.array().colwise()*Eigen::pow(t,Eigen::ArrayXd::LinSpaced(p.rows(),0,p.rows()-1))).colwise().sum();
  }
};//End of class SubscribeAndPublish


int main(int argc, char** argv)
{
  ros::init(argc, argv, "filter_node");
  
  SubscribeAndPublish sap;
  
  ros::spin();
  return 0;
}


/*
 *  Player - One Hell of a Robot Server
 *  Copyright (C) 2000  Brian Gerkey   &  Kasper Stoy
 *                      gerkey@usc.edu    kaspers@robotics.usc.edu
 *
 *  This library is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 2.1 of the License, or (at your option) any later version.
 *
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this library; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
///////////////////////////////////////////////////////////////////////////
//
// Desc: AMCL Marler routines
// Author: Andrew Howard
// Date: 6 Feb 2003
// CVS: $Id: amcl_marker.cc 7057 2008-10-02 00:44:06Z gbiggs $
//
///////////////////////////////////////////////////////////////////////////

#include <sys/types.h>  // required by Darwin
#include <math.h>
#include <stdlib.h>
#include <assert.h>
#include <unistd.h>
#include <vector>
#include <iostream>
#include <numeric>
#include <algorithm>
#include <amcl_hybrid/pixels_corners.h>
#include <amcl_hybrid/pixels_cloud.h>
#include <amcl_hybrid/coeff_sensor.h>
#include <amcl_hybrid/marker_error.h>

#include "amcl/sensors/amcl_marker.h"

using namespace amcl;

////////////////////////////////////////////////////////////////////////////////
// Default constructor
AMCLMarker::AMCLMarker(int simulation) : AMCLSensor()

{
  // this->map = map;
  this->simulation = simulation;
  // #% lo voy a comentar porque he generado un callback que se subscribe al topic de infoCamera
  // this->LoadCameraInfo();

  return;
}

AMCLMarker::~AMCLMarker()
{
  if (temp_obs.size() > 0)
  {
    temp_obs.clear();
  }
}

void AMCLMarker::SetModelLikelihoodField(double z_hit, double z_rand, double sigma_hit, double landa,
                                         double marker_coeff)
{
  this->model_type = MARKER_MODEL_LIKELIHOOD;
  this->z_hit = z_hit;
  this->z_rand = z_rand;
  this->sigma_hit = sigma_hit;
  this->landa = landa;
  this->marker_coeff = marker_coeff;
}

////////////////////////////////////////////////////////////////////////////////
// Apply the marker sensor model
bool AMCLMarker::UpdateSensor(pf_t* pf, AMCLSensorData* data)
{
  // cout<<"update sensor marker cpp"<<endl;
  if (this->model_type == MARKER_MODEL_LIKELIHOOD)
    pf_update_sensor(pf, (pf_sensor_model_fn_t)ObservationLikelihood, data);
  return true;
}

// Determine the probability for the given pose
double AMCLMarker::ObservationLikelihood(AMCLMarkerData* data, pf_sample_set_t* set)
{
  std::cout << "\n###### in particle filter MARKERS ######" << std::endl;

  AMCLMarker* self;

  pf_sample_t* sample;
  pf_vector_t pose;
  pf_vector_t hit;
  double total_weight;
  double pz, p;
  std::vector<float> z;
  self = (AMCLMarker*)data->sensor;  //
  std::vector<Marcador> observation = data->markers_obs;
  std::cout << observation.size() << " markers received" << std::endl;
  // cout<<"landa in likelihood"<<self->landa<<endl;
  total_weight = 0.0;
  float gaussian_norm = 1 / (sqrt(2 * M_PI * self->sigma_hit * self->sigma_hit));
  int valid_msg = 0;

  amcl_hybrid::pixels_cloud aux_pixels_cloud;
  // ###### Un ciclo por cada muestra de la nube de puntos (sample)
  float total_error_marker = 0.0;
  for (int i = 0; i < set->sample_count; i++)
  {
    // std::cout << "\nSample " << i << std::endl;

    sample = set->samples + i;
    pose = sample->pose;
    p = 1.0;

    // Initialize parameters
    double z_hit_denom = 2 * self->sigma_hit * self->sigma_hit;  // 2*sigma_hit²
    // sqrt(2) beacuse of the normalization with height and width of image.
    double z_rand_mult = 1.0 / sqrt(2);

    geometry_msgs::Pose sample_pose;  // definimos la muestra de tipo geometry_msgs/Point position,
    tf::Quaternion quat;
    geometry_msgs::Quaternion quat_msg;  // definimos un objeto de tipo geometry_msgs/Quaternion orientation
    sample_pose.position.x = pose.v[0];  // sample_pose toma los valores del la muestra
    sample_pose.position.y = pose.v[1];
    sample_pose.position.z = 0.0;  // es 0 porque el amcl trabaja sobre el plano 2D

    // #% El ángulo puede venir con mas de 2PI radiales, lo que hacemos es transformarlo en un valor entre 0 y 2PI
    pose.v[2] = fmod(pose.v[2], 2 * M_PI);  // fomd-> devuelve el resto(remainder) en coma flotante
    if (pose.v[2] < 0)
    {
      pose.v[2] = pose.v[2] + 2 * M_PI;
    }
    // cout<<pose.v[2]<<endl;
    quat.setRPY(0, 0, pose.v[2]);
    tf::quaternionTFToMsg(quat, quat_msg);
    sample_pose.orientation = quat_msg;
    
    // *** ADAPT MARKER MAP POINTS TO THE PARTICLE ***
    bool print_steps = false;

    // STEP 1. Proyect all points relative to the camera, points should maintain marker ownership
    // Points are stored in the field map[n].ReltoCam
    self->CalculateRelativePose(sample_pose, self);

    if(print_steps){
    std::cout << "ReltoCam first rectangle in map:" << std::endl;
    for(const auto &point : self->map[0].ReltoCam) {
        std::cout << "  Point: (" << point.x << ", " << point.y << ", " << point.z << ")" << std::endl;
    }}

    // STEP 2. Filter points with the implicit equation of the camera, discard markers where all points are not
    // STEP    maintained
    std::vector<size_t> filtered_marker_indices = self->FilterPointsByFOV(sample_pose, self);

    if(print_steps){
    std::stringstream ss;
    ss << "Filter Indices: [";
    for(const auto &index : filtered_marker_indices) {
        ss <<  index << ", ";
    }
    ss << "]\n";
    std::cout << ss.str();
    }

    // STEP 3. Project those points to the image plane
    std::vector<std::vector<cv::Point2d>> projected_markers(filtered_marker_indices.size());
    std::transform(
        filtered_marker_indices.begin(), filtered_marker_indices.end(), projected_markers.begin(),
        [self](size_t j /*j: filtered marker index*/) { return self->projectPoints(self->map[j].ReltoCam); });

    if(print_steps){
    for (const auto& marker : projected_markers) {
        std::cout << "Projected" << std::endl;
        for (const auto& point : marker) {
            std::cout << "  Point: (" << point.x << ", " << point.y << ")" << std::endl;
        }
    }}

    // STEP 4. For each map marker, for each detected marker calculate the error, keep the detected marker with min
    // STEP    error, threshold error
    // projected_markers are from the map, observations are from the detector, we want for each marker to iterate
    // throught eh obserations and find that with min distance to the corners. The result will be according to the
    // specified formula and in the form of a vector of floats (z) and we need to accumulate it into ztot and add it to
    // total_marker_error, apply the formula and add it to pz and p (pz³)
    std::vector<std::pair<size_t, float>> ztot_vector(filtered_marker_indices.size());
    std::transform(projected_markers.begin(), projected_markers.end(), ztot_vector.begin(),
                   [&self, &observation](std::vector<cv::Point2d> projected_marker) {
                     return self->calculateError(observation, projected_marker);  // TODO: Verify adapt the function
                   });

    if(print_steps){
    for (const auto& pair : ztot_vector) {
        std::cout << "Ztot: (" << pair.first << ", " << pair.second << ")" << std::endl;
    }}

    float min_error = 1.0f;
    std::for_each(ztot_vector.begin(), ztot_vector.end(), [&](std::pair<size_t, float> ztot) {
      if (ztot.second < min_error) {
        total_error_marker += ztot.second;
        p += std::pow(self->landa * exp(-self->landa * ztot.second), 3);
      }
    });

    // Show info position and orientation of each pointcloud
    // std::cout << "  Pos-> x:" << sample_pose.position.x << "  y:" << sample_pose.position.y;
    // std::cout << "  yaw:" << pose.v[2] << "("<< pose.v[2]*(180/M_PI) << "º)" << std::endl;
    // cout << " Quat-> x:" << sample_pose.orientation.x << "  y:" << sample_pose.orientation.y << "  z:" <<
    // sample_pose.orientation.z << "  w:" << sample_pose.orientation.w << endl;
    //* for (int j = 0; j < observation.size(); j++)
    //* {
      // std::cout << "Marker " << j;

      // Calculate projection of marker corners
      // Sabemos exactamente donde está la marca respecto al mundo. Ahora según la posición de la particula, vamos a
      // calcular la posición que tendría los corners en 3D respecto a la cámara # entro una vez por cada corner
      /*relative_to_cam{
          (fila 0:) X_corner0, Y_corner0, Z_corner0;
          (fila 1:) X_corner1, Y_corner1, Z_corner1;
          (fila 2:) X_corner2, Y_corner2, Z_corner2;
          (fila 3:) X_corner3, Y_corner3, Z_corner3}
          Son las posiciones de cada uno de los corners respecto al centro de la camara
      */

      //* std::vector<geometry_msgs::Point> relative_to_cam=self->CalculateRelativePose(detected_from_map[j],
      // sample_pose);

      // cout<<"after relative pose"<<endl;
      // std::vector<cv::Point2d> projection;
      // cout<<"simu"<<endl;
      //  obtenemos la proyección de los corners reales del marker recibido 3D a 2D de la imagen
      // projection = self->projectPoints(relative_to_cam);
      // projection (width, height) refer to top/left
      // std::cout << "\n   Pin-hole projection: " << std::endl;
      // std::cout << "      Corners" << " -> " << projection << std::endl;

      //* aux_pixels_cloud.pixels_cloud.push_back(self->send_pixels_corners);
      // Calculate mean error in pixels
      // Posición en pixeles de los corner observados en la imagen del detector
      //* std::vector<cv::Point2f> Puntos = observation[j].getMarkerPoints();

      // Compute probability for every corner
      // ### observation[j].getMarkerPoints() Son la position real de cada corner en la imagen
      // ### projection son la posición teorica que tendrían cada corner de cada punto segun la posición del robot
      // despues de utilizar pin-hole
      //* z = self->calculateError(observation[j].getMarkerPoints(),
      //*                          projection);  // saltamos a la función que lo hace. Retornamos el peso de la
      // partícula
      // z es un vector de float de el error de cada partícula
      // cout<<"despues de error"<<endl;
      //* float ztot = std::accumulate(z.begin(), z.end(),
      //*                              0.0);  // hace la suma de todos el rango (rango_incial, rango_final, valor
      // inicial)
      //* total_error_marker += ztot;
      // cout<<"despues de sumar"<<endl;
      // for (int i=0;i<4;i++){
      //* pz = 0.0;
      // Opción1:Gaussian model
      // pz+=self->z_hit*exp(-(z[i]*z[i]) / z_hit_denom);
      // Random measurements
      // pz+=self->z_rand*z_rand_mult;
      //  cout<<"pz: "<<pz<<endl;
      // p+=pz*pz*pz;
      // Opción 2:Distribución exponencial (Humanoid P12)
      // pz+=z[i];
      //* pz += self->landa * exp(-self->landa * ztot);
      //* p += pz * pz * pz;
      // cout << " pz:" << pz << ", p: " << p << endl;
      // }
      /*  if (pz>1.0){
          cout<<"mayor"<<endl;
      }*/
    //* }
    
    sample->weight *= p;
    total_weight += sample->weight;
    // cout << "Sample " << i << " -> " << sample->weight<<endl;
  }  // un ciclo por
  self->send_pixels_cloud = aux_pixels_cloud;
  // #% This one I have added to use marker_coeff;
  total_weight *= self->marker_coeff;
  std::cout << "*** Calculated total_weight Marker: " << total_weight << " (coeff: " << self->marker_coeff << " )"
            << std::endl;
  amcl_hybrid::coeff_sensor fail_marker_msg;
  fail_marker_msg.header.stamp = ros::Time::now();
  fail_marker_msg.coeff.data = total_weight;
  fail_marker_msg.sensor.data = "marker";
  self->pub_coeff_marker.publish(fail_marker_msg);
  amcl_hybrid::marker_error msg_marker_error;
  msg_marker_error.total_error.data = total_error_marker / float(set->sample_count * observation.size());
  self->pub_marker_error.publish(msg_marker_error);
  return (total_weight);
}

double calculateDistance(cv::Point2f observation_point, cv::Point2d marker_point) {
    return std::sqrt(std::pow(marker_point.x - observation_point.x, 2) + std::pow(marker_point.y - observation_point.y, 2));
}

// ##### esta función es la que calcula el error sabiendo donde esta la proyección teórica (la recibida) y la del mapa
std::pair<size_t, float> AMCLMarker::calculateError(std::vector<Marcador>& observation,
                                                    std::vector<cv::Point2d> projected_map_marker)
{
  // ### projected_observation = observation[j].getMarkerPoints(), Son la position observada de cada corner en la imagen
  // recibida
  // ### projected_map_marker = projection, son la posición teorica que tendrían cada corner del marker detectado en la imagen
  // después de aplicar pin_hole
  //* Find the marker with minimum error and return that marker's id and error
  std::vector<float> observation_errors(observation.size());
  for (size_t j = 0; j < observation.size(); j++)
  {
    ROS_DEBUG_STREAM("PROCESSING MARKER " << j);
    std::vector<cv::Point2f> projected_observation = observation[j].getMarkerPoints();

    // normalizing error with width and height of image.
    float errorv = 0;  // vector de error cálculado para cada corner

    // cout << "   Error:" << endl;
    for (int i = 0; i < 4; i++) //corner in map_marker
    {
      float errorx, errory;
      float error = 0.0;
      std::vector<float> corner_errors(projected_observation.size());

      for (int j = 0; j < projected_observation.size(); j++) //corner in observation
      {
        corner_errors[j] = calculateDistance(projected_observation[j], projected_map_marker[i]);
      }
      //find the index of the element with minimum corner error, and remove it from projected observation
      int min_corner_error_index = std::distance(corner_errors.begin(), std::min_element(corner_errors.begin(), corner_errors.end()));
      cv::Point2f matched_corner = projected_observation[min_corner_error_index];
      projected_observation.erase(projected_observation.begin() + min_corner_error_index);
      

      errorx = abs(projected_map_marker[i].x - matched_corner.x) /
               image_width;  // |Posicion_real - Posición_teorica| / ancho imagen
      errory = abs(projected_map_marker[i].y - matched_corner.y) / image_height;
      error = sqrt((errorx * errorx) + (errory * errory));  // error = error + sqrt((error en x del corner)² + (error
                                                             // en y del corner)²)
      errorv += error;
      // Show info Error
      /*std::cout << "_______" << std::endl;
      cout << "      Corner " << i << " pos-> map: " << projected_map_marker[i].x <<", " << projected_map_marker[i].y << " detected:
      " << projected_observation[i].x << ", " << projected_observation[i].y << endl; cout << "         error " << "-> [" <<
      errorx << ", " << errory << "] " << "Total: " << error << endl ;*/
    }

    observation_errors[j] = errorv;
  }

  size_t min_error_index =
      std::distance(observation_errors.begin(), std::min_element(observation_errors.begin(), observation_errors.end()));

  float min_error = observation_errors[min_error_index];
  // if (min_error > 1) //TODO: set this threshold to a reasonable value, curretly it is not filtering
  // {
  //   ROS_WARN_STREAM("Failed to detect Marker -> error is higher than 1: " << min_error ); 
  // }
  return std::make_pair(min_error_index, min_error);  // retornamos el id y error minimo
}

// Entra una vez por cada marker encontrado
std::vector<cv::Point2d> AMCLMarker::projectPoints(std::vector<geometry_msgs::Point> cam_center_coord)
{
  /*cam_center_coord{
      (fila 0:) X_corner0, Y_corner0, Z_corner0;
      (fila 1:) X_corner1, Y_corner1, Z_corner1;
      (fila 2:) X_corner2, Y_corner2, Z_corner2;
      (fila 3:) X_corner3, Y_corner3, Z_corner3}
  Son las posiciones de cada uno de los corners respecto al centro de la camara
  */
  // cout<<"entra en relative"<<endl;
  geometry_msgs::PointStamped cam_center_coord_st, cam_trans_coord_st;
  std::vector<cv::Point2d> Pixels;
  amcl_hybrid::pixels_corners aux_pixels_corners;
  geometry_msgs::Point32 pixel_corner;
  for (int i = 0; i < cam_center_coord.size(); i++)
  {  // size es 4 que es igual al número de corner
    cam_center_coord_st.point = cam_center_coord[i];
    cv::Point2d Pixel;
    cv::Point3d Coord;
    tf2::doTransform(cam_center_coord_st, cam_trans_coord_st, tf_cameras[0]);
    Coord.x = cam_trans_coord_st.point.x;
    Coord.y = cam_trans_coord_st.point.y;
    Coord.z = cam_trans_coord_st.point.z;
    Pixel = this->pin_model.project3dToPixel(Coord);
    // if (Pixel.x > pin_model.cameraInfo().width || Pixel.y > pin_model.cameraInfo().height) continue; // TODO: BUG, REMOVE WHOLE MARKER
    pixel_corner.x = Pixel.x;
    pixel_corner.y = Pixel.y;
    pixel_corner.z = 0;
    Pixels.push_back(Pixel);
    aux_pixels_corners.pixels_corners.push_back(pixel_corner);
  }
  // cout<<"sale"<<endl;
  this->send_pixels_corners = aux_pixels_corners;
  return Pixels;
}

void AMCLMarker::LoadCameraInfo(void)
{
  // yo tengo que quitar todo esto y leer mi topic de camera info
  //_____________________________________________________________________________________________________
  sensor_msgs::CameraInfo cam_inf_ed;
  // if (this->simulation == 1){
  cam_inf_ed.header.frame_id = "xtion_rgb_optical_frame";
  cam_inf_ed.height = 480;
  cam_inf_ed.width = 640;
  cam_inf_ed.distortion_model = "plumb_bob";
  double Da[5] = { 0.02019943683275026, -0.08339482869293142, -0.004265862094806434, 0.0041277952286131285, 0.0 };
  boost::array<double, 9ul> K = { { 515.8668782266329, 0.0, 332.2750817773881, 0.0, 516.2546881081752, 237.2817318673198, 0.0, 0.0, 1.0 } };
  boost::array<double, 9ul> R = { { 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0 } };
  boost::array<double, 12ul> P = { { 513.1033325195312, 0.0, 335.3405633298644, 0.0, 0.0, 516.9007568359375, 235.73558310651788, 0.0, 0.0, 0.0, 1.0, 0.0 } };
  std::vector<double> D(Da, Da + (sizeof(Da) / sizeof(Da[0])));
  cam_inf_ed.D = D;
  cam_inf_ed.K = K;
  cam_inf_ed.R = R;
  cam_inf_ed.P = P;
  cam_inf_ed.binning_x = 0.0;
  cam_inf_ed.binning_y = 0.0;
  cam_inf_ed.roi.height = 0;
  cam_inf_ed.roi.width = 0;
  //}
  // if (this->simulation == 0){
  // camMatrix = cv::Mat(3, 3, CV_32F);
  // camMatrix.at<float>(0, 0) = 8.5101024687735935e+02;
  // camMatrix.at<float>(0, 1) = -2.2255059056366439e-01;
  // camMatrix.at<float>(0, 2) = 6.5571465382877625e+02;
  // camMatrix.at<float>(1, 0) = 0.0;
  // camMatrix.at<float>(1, 1) = 8.5170243585411265e+02;
  // ;
  // camMatrix.at<float>(1, 2) = 5.1216084358475405e+02;
  // camMatrix.at<float>(2, 0) = 0.0;
  // camMatrix.at<float>(2, 1) = 0.0;
  // camMatrix.at<float>(2, 2) = 1.0;

  // distCoeff = cv::Mat(4, 1, CV_32F);
  // distCoeff.at<float>(0, 0) = -4.2648301140911193e-01;
  // distCoeff.at<float>(1, 0) = 3.1105618959437248e-01;
  // distCoeff.at<float>(2, 0) = -1.3775384616268102e-02;
  // distCoeff.at<float>(3, 0) = -1.9560559208606078e-03;
  // distCoeff.at<float>(3, 0) = 0;

  // xi = 1.5861076761699640e+00;

  // }

  this->pin_model.fromCameraInfo(cam_inf_ed);
  //________________________________________________________________________________________________________________
}

// #%
void AMCLMarker::LoadCameraInfo2(const sensor_msgs::CameraInfoConstPtr& cam_info)
{
  this->pin_model.fromCameraInfo(cam_info);
  this->cam_inf_ed = *cam_info;
}

// #entramos una vez por cada marker
void AMCLMarker::CalculateRelativePose(geometry_msgs::Pose CamaraMundo, amcl::AMCLMarker* self)
{
  // Marcador = info de la marca del archivo yaml position_marker
  // CamaraMundo = posición de la cámara (es la posición de cada muestra de la nube de puntos) respecto del mundo
  // (translación y orientazión) Pose CAM;
  tf::Transform MundTrob, invMundTrob, RobTCam, invRobotTCam;
  tf::Quaternion RotCam;
  /*MundTrob = transform from world to robot
    invMundTrob = inverse of transform from world to robot
    RobTCam = transfor from robot to camera
    invRobotTCam = transfor from robot to camera
    RotCam = camera rotation refered to robot
  */

  // From Robot base to camera
  RotCam.setRPY(-M_PI / 2, 0, -M_PI / 2);  // cambiamos el eje de coordenadas del mundo al la camara
  /*Eje X toward left, Eje Y toward dowm, Eje Z forward
   */
  // RobTCam.setOrigin(tf::Vector3(0,0,height_pos_camera_link_));
  RobTCam.setOrigin(tf::Vector3(0, 0, this->height_center_camera));
  RobTCam.setRotation(RotCam);
  tf::Quaternion QMundRCam(CamaraMundo.orientation.x, CamaraMundo.orientation.y, CamaraMundo.orientation.z,
                           CamaraMundo.orientation.w);
  tf::Vector3 Trasl1(CamaraMundo.position.x, CamaraMundo.position.y, CamaraMundo.position.z);
  // cout<<"after transformation"<<endl;
  // From World to Robot
  MundTrob.setRotation(QMundRCam);
  MundTrob.setOrigin(Trasl1);
  // Inverse the transformation--> inversa del mundo a la camara
  invRobotTCam = RobTCam.inverse();
  invMundTrob = MundTrob.inverse();
  geometry_msgs::TransformStamped MundTrobSt, RobotTCamSt;
  MundTrobSt.header.frame_id = "ground_plane__link";
  MundTrobSt.child_frame_id = "EstimatedPose";
  RobotTCamSt.header.frame_id = "EstimatedPose";
  RobotTCamSt.child_frame_id = "camera_link";
  transformTFToMsg(MundTrob, MundTrobSt.transform);
  transformTFToMsg(RobTCam, RobotTCamSt.transform);
  // this->br_marker.sendTransform(MundTrobSt);
  // this->br_marker.sendTransform(RobotTCamSt);

  // Pose Transformation
  geometry_msgs::TransformStamped invMundTrobStamped, invRobotTCamSt;
  transformTFToMsg(invMundTrob, invMundTrobStamped.transform);
  transformTFToMsg(invRobotTCam, invRobotTCamSt.transform);
  std::vector<geometry_msgs::Point> PoseWorld;  // geometry_msgs::Point -> [float64 x, float64 y, float64 z]
  // std::vector<geometry_msgs::Transform> Corners = Marca.getTransformCorners();
  // cout<<"antes de get pose world"<<endl;
  for (uint j = 0; j < self->map.size(); j++)
  {
    Marcador& Marker = self->map[j];  // taken by reference, we are modifying the map itself
    Marker.ReltoCam.clear();

    PoseWorld = Marker.getPoseWorld();  // es un vector de las posiciones alamacenadas de los 4 corners en 3D que fueron
                                        // previamente calculadas
    // std::cout << "  , corners position:" << std::endl;
    for (int i = 0; i < 4; i++)  // un bucle para cada corner
    {
      /* What it do is: each corner position en 3D respecto al mundo "PoseWorld", lo pasa a la posición respecto a la
         cámara para posteriormente poder utilizar pin-hole
      */
      geometry_msgs::PointStamped CornerRelPose, Inter, WorldPose;
      WorldPose.point = PoseWorld[i];
      tf2::doTransform(WorldPose, Inter, invMundTrobStamped);  // transformada del (mundoToRobot)⁻1 : Robot -> World
      tf2::doTransform(Inter, CornerRelPose, invRobotTCamSt);  // transformada del (RobotToCamera)-1 : Camera -> Robot
      Marker.ReltoCam.push_back(CornerRelPose.point);          // cada corner lo añade a un vector
                                                               /*RelativaCorners{
                                                               (fila 0:) X_corner0, Y_corner0, Z_corner0;
                                                               (fila 1:) X_corner1, Y_corner1, Z_corner1;
                                                               (fila 2:) X_corner2, Y_corner2, Z_corner2;
                                                               (fila 3:) X_corner3, Y_corner3, Z_corner3}
                                                               Son las posiciones de cada uno de los corners respecto al centro de la camara
                                                               */
      // Show positions and orientations of marker refer to world and camera(each particle)
      // std::cout << "   World XYZ " << i << "  (" << PoseWorld[i].x << ", " << PoseWorld[i].y << ", " <<
      // PoseWorld[i].z << ")"<< std::endl; std::cout << "   Rela* XYZ " << i << "  (" << CornerRelPose.point.x << ", "
      // << CornerRelPose.point.y << ", " << CornerRelPose.point.z << ")" << std::endl;
    }
  }
}

// Method to filter points to project from the map to a camera based on the implicit equation of the cone
std::vector<size_t> AMCLMarker::FilterPointsByFOV(geometry_msgs::Pose CamaraMundo, amcl::AMCLMarker* self)
{
  std::vector<size_t> filtered_marker_indices;  // vector of indices
  const double dfov = std::sqrt(std::pow(self->pin_model.fovX(), 2) +
                                std::pow(self->pin_model.fovY(), 2));  // use fovD for including all possible points

  for (size_t j = 0; j < self->map.size(); j++)
  {
    const std::vector<geometry_msgs::Point> corners = self->map[j].ReltoCam;
    std::vector<bool> corners_accepted;

    for (size_t i = 0; i < corners.size(); i++)
    {
      const geometry_msgs::Point corner = corners[i];
      // STEP 1. Discard marker if any points are behind the camera: x<0
      if (corner.x < 0)
        break;  // do not bother calculating more corners if a single one fails

      // STEP 2. Keep points inside the cone: y^2 + x^2 <= (tan(dfov / 2))^2 * z^2
      if (std::pow(corner.y, 2) + std::pow(corner.x, 2) <= std::pow(std::tan(dfov / 2), 2) * std::pow(corner.z, 2))
      {
        corners_accepted.push_back(true);
      } else {
        break;  // do not bother calculating more corners if a single one fails
      }
    }

    // STEP 3. Only markers with all corners inside the cone shall be projected
    if (corners_accepted.size() == corners.size())
    {
      filtered_marker_indices.push_back(j);
    }
  }

  return filtered_marker_indices;
}

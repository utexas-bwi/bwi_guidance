/**
 * @file /include/bwi_exp1_visualizer/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef bwi_exp1_visualizer_QNODE_HPP_
#define bwi_exp1_visualizer_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <string>
#include <QThread>
#include <QStringListModel>

#include <bwi_mapper/map_loader.h>
#include <bwi_mapper/graph.h>
#include <bwi_exp1/experiment.h>
#include <bwi_exp1/users.h>
#include <bwi_exp1/odometry.h>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace bwi_exp1_visualizer {

/*****************************************************************************
** Class
*****************************************************************************/

  class QNode : public QThread {
    Q_OBJECT

    public:
      QNode(int argc, char** argv);
      virtual ~QNode();
      bool init();
      void run();
      void updateFrame(int time_step);

      std::vector<std::string> experiment_box_strings_;
      std::vector<std::string> user_box_strings_;
      float max_experiment_time_;
      int max_time_steps_;
      float current_experiment_time_;
      int current_time_step_;
      float time_step_;
      cv::Mat generated_image_;

      int current_experiment_index_;
      int current_user_index_;

    public slots:
      void on_timeSlider_valueChanged(int value);
      void on_autoPlayBox_toggled(bool checked);
      void on_experimentBox_currentIndexChanged(int index);
      void on_userBox_currentIndexChanged(int index);

    signals:
      void incrementTime();
      void updateFrameInfo();
      void rosShutdown();

    private:

      cv::Point toImage(const cv::Vec2f& pt);

      /* UI Elements */
      bool autoplay;

      /* ROS Elements and paramters*/
      int init_argc;
      char** init_argv;
      std::string map_file_;
      std::string graph_file_;
      std::string experiment_file_;
      std::string data_directory_;
      std::string users_file_;

      /* Experiment and graph information */
      boost::shared_ptr<bwi_mapper::MapLoader> mapper_;
      bwi_mapper::Graph graph_;
      bwi_exp1::Experiment experiments_;
      std::vector<bwi_exp1::User> users_;
      std::vector< std::vector<size_t> > user_box_to_idx_map_;
      std::vector< std::vector< 
        std::vector<bwi_exp1::LocationStamped> > > odometry_;

  };

}  // namespace bwi_exp1_visualizer

#endif /* bwi_exp1_visualizer_QNODE_HPP_ */

/**
 * @file /include/clingo_interface/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef clingo_interface_QNODE_HPP_
#define clingo_interface_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <string>
#include <QThread>
#include <QStringListModel>
#include <clingo_helpers/ClingoInterface.h>

#include <opencv/cv.h>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace clingo_interface {

/*****************************************************************************
** Class
*****************************************************************************/

  class QNode : public QThread {
    Q_OBJECT
    public:
      QNode(int argc, char** argv );
      virtual ~QNode();
      bool init();
      void run();

      /* Service callback */
      bool clingoInterfaceHandler(clingo_helpers::ClingoInterface::Request &req,
          clingo_helpers::ClingoInterface::Response &resp);

      /* Display stuff */
      QString display_text_;
      bool button1_enabled_;
      QString button1_text_;
      bool button2_enabled_;
      QString button2_text_;
      cv::Mat generated_image_;

    signals:
      void rosShutdown();

    private:
      int init_argc;
      char** init_argv;

  };

}  // namespace clingo_interface

#endif /* clingo_interface_QNODE_HPP_ */

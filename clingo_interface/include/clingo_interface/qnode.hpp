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

    signals:
      void rosShutdown();

    private:
      int init_argc;
      char** init_argv;

  };

}  // namespace clingo_interface

#endif /* clingo_interface_QNODE_HPP_ */

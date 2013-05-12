/**
 * @file /include/clingo_interface/main_window.hpp
 *
 * @brief Qt based gui for clingo_interface.
 *
 * @date November 2010
 **/
#ifndef clingo_interface_MAIN_WINDOW_H
#define clingo_interface_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui/QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"

/*****************************************************************************
** Namespace
*****************************************************************************/

namespace clingo_interface {

  /*****************************************************************************
   ** Interface [MainWindow]
   *****************************************************************************/
  /**
   * @brief Qt central, all operations relating to the view part here.
   */
  class MainWindow : public QMainWindow {
    Q_OBJECT

    public:
      MainWindow(int argc, char** argv, QWidget *parent = 0);
      ~MainWindow();

    public slots:

      void updateFrameInfo();

    private:
        Ui::MainWindow ui;
        QNode qnode;
  };

}  // namespace clingo_interface

#endif // clingo_interface_MAIN_WINDOW_H

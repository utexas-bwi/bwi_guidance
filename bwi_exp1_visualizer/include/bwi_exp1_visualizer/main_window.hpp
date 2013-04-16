/**
 * @file /include/bwi_exp1_visualizer/main_window.hpp
 *
 * @brief Qt based gui for bwi_exp1_visualizer.
 *
 * @date November 2010
 **/
#ifndef bwi_exp1_visualizer_MAIN_WINDOW_H
#define bwi_exp1_visualizer_MAIN_WINDOW_H

/*****************************************************************************
 ** Includes
 *****************************************************************************/

#include <QtGui/QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"

/*****************************************************************************
 ** Namespace
 *****************************************************************************/

namespace bwi_exp1_visualizer {

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

      /* Manually triggered slots (from qnode) */
      void on_updateFrameInfo();
      void on_incrementTime();

    private:
      Ui::MainWindow ui;
      QNode qnode;
  };

}  // namespace bwi_exp1_visualizer

#endif // bwi_exp1_visualizer_MAIN_WINDOW_H

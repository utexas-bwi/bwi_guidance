/**
 * @file /include/bwi_guidance_visualizer/main_window.hpp
 *
 * @brief Qt based gui for bwi_guidance_visualizer.
 *
 * @date November 2010
 **/
#ifndef bwi_guidance_visualizer_MAIN_WINDOW_H
#define bwi_guidance_visualizer_MAIN_WINDOW_H

/*****************************************************************************
 ** Includes
 *****************************************************************************/

#include <QtGui/QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"

/*****************************************************************************
 ** Namespace
 *****************************************************************************/

namespace bwi_guidance_visualizer {

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
      void updateFrameInfo();
      void incrementTime();

    private:
      Ui::MainWindow ui;
      QNode qnode;
  };

}  // namespace bwi_guidance_visualizer

#endif // bwi_guidance_visualizer_MAIN_WINDOW_H

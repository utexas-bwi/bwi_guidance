/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date February 2011
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include "../include/clingo_interface_gui/main_window.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace clingo_interface_gui {

  using namespace Qt;

  /*****************************************************************************
   ** Implementation [MainWindow]
   *****************************************************************************/

  MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
    : QMainWindow(parent)
      , qnode(argc,argv) {

    /* Setup auto slots with this function call */
    ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.

    /* Start the node */
    qnode.init();

    QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));
    QObject::connect(&qnode, SIGNAL(updateFrameInfo()), 
        this, SLOT(updateFrameInfo()));
    QObject::connect(ui.button1, SIGNAL(valueChanged(int)), 
        &qnode, SLOT(on_timeSlider_valueChanged(int)));

    ui.button1->setText("(none)");
    ui.button1->setEnabled(false);
    ui.button2->setText("(none)");
    ui.button2->setEnabled(false);

  }

  MainWindow::~MainWindow() {}

  /*****************************************************************************
   ** Implementation [Slots]
   *****************************************************************************/
  
  void MainWindow::updateFrameInfo() {

    /* Update Text */
    ui.textDisplay->setText(qnode.display_text_.c_str());

    /* Update Buttons */
    if (qnode.button1_enabled_) {
      ui.button1->setText(qnode.button1_text_.c_str());
      ui.button1->setEnabled(true);
    } else {
      ui.button1->setText("(none)");
      ui.button1->setEnabled(false);
    }

    if (qnode.button2_enabled_) {
      ui.button2->setText(qnode.button2_text_.c_str());
      ui.button2->setEnabled(true);
    } else {
      ui.button2->setText("(none)");
      ui.button2->setEnabled(false);
    }

    /* Update image */
    float image_height = qnode.generated_image_.rows;
    float image_width = qnode.generated_image_.cols;
    float screen_height = ui.localMap->frameRect().size().height();
    float screen_width = ui.localMap->frameRect().size().width();

    float scale_height = screen_height / image_height;
    float scale_width = screen_width / image_width;
    float scale = std::min(scale_height, scale_width);

    cv::Mat rgb;
    cv::resize(qnode.generated_image_, rgb, cv::Size(), scale, scale, 
        cv::INTER_CUBIC);

    /* http://stackoverflow.com/questions/5026965/how-to-convert-an-opencv-cvmat-to-qimage */
    QImage dest(rgb.cols, rgb.rows, QImage::Format_ARGB32);
    for (int y = 0; y < rgb.rows; ++y) {
      const cv::Vec3b *rgbrow = rgb.ptr<cv::Vec3b>(y);
      QRgb *destrow = (QRgb*)dest.scanLine(y);
      for (int x = 0; x < rgb.cols; ++x) {
        destrow[x] = qRgba(rgbrow[x][2], rgbrow[x][1], rgbrow[x][0], 255);
      }
    }
    ui.localMap->setPixmap(QPixmap::fromImage(dest));

  }
}  // namespace clingo_interface_gui


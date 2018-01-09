#ifndef PROJECT_OTHERPANEL_H
#define PROJECT_OTHERPANEL_H

#include <ros/ros.h>
#include <rviz/panel.h>
#include <rr_platform/chassis_state.h>
#include <QLabel>
#include <QtGui>
#include <QPainter>
#include <math.h>
#include <QString>


/*
 * All of our panels need to be under the rr_rviz_plugins namespace.
 */
namespace rr_rviz_plugins {

/*
 * Each panel is a subclass of rviz::Panel
 */
class OtherPanel : public rviz::Panel {




/*
 * rviz is based on QT, so our panels need to be QObjects, which requires this macro keyword.
 */
Q_OBJECT
public:

    /**
     * This is a standard QWidget constructor.
     * @param parent The parent widget, which will be responsible for the lifetime of this widget.
     */
    OtherPanel(QWidget *parent = 0);

protected:
    /*
     * stuff for painting
     */
    int height = 200;
    int width = 200;
    int maxSpeed = 3;
    void paintEvent(QPaintEvent*);
    /*
     * Be sure to make any publishers / subscribers members of the class. This will keep them alive throughout
     * the lifetime of the widget.
     */
    ros::Subscriber chassis_subscriber;

    /**
     * Declare any ROS callbacks you need here. Be sure to create a parameter for any UI elements you need to update.
     * @param msg The ROS message that triggers this callback.
     * @param label A QT label whose text we will update based on the message contents.
     */
    void chassis_callback(const rr_platform::chassis_stateConstPtr &msg, QLabel *speedlabel);

    /**
     * If you need to paint custom graphics on your panel, uncomment and implement the paintEvent method.
     * You can find out more about this method here: http://doc.qt.io/qt-5/qwidget.html#paintEvent
     */
//    void paintEvent(QPaintEvent *event) override;

};

}

#endif //PROJECT_OTHERPANEL_H

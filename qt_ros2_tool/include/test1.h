/********************************************************************************
** Form generated from reading UI file 'test1.ui'
**
** Created by: Qt User Interface Compiler version 5.15.3
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef TEST1_H
#define TEST1_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QSlider>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QWidget *centralwidget;
    QPushButton *pushButton;
    QWidget *verticalLayoutWidget;
    QVBoxLayout *verticalLayout;
    QHBoxLayout *horizontalLayout_4;
    QLabel *label_4;
    QLineEdit *LE_x;
    QSlider *HS_x;
    QHBoxLayout *horizontalLayout_5;
    QLabel *label_5;
    QLineEdit *LE_y;
    QSlider *HS_y;
    QHBoxLayout *horizontalLayout_3;
    QLabel *label_3;
    QLineEdit *LE_z;
    QSlider *HS_z;
    QHBoxLayout *horizontalLayout_6;
    QLabel *label_6;
    QLineEdit *LE_yaw;
    QSlider *HS_yaw;
    QHBoxLayout *horizontalLayout;
    QLabel *label;
    QLineEdit *LE_pitch;
    QSlider *HS_pitch;
    QHBoxLayout *horizontalLayout_2;
    QLabel *label_2;
    QLineEdit *LE_roll;
    QSlider *HS_roll;
    QPushButton *PB_MV;
    QStatusBar *statusbar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QString::fromUtf8("MainWindow"));
        MainWindow->resize(800, 600);
        centralwidget = new QWidget(MainWindow);
        centralwidget->setObjectName(QString::fromUtf8("centralwidget"));
        pushButton = new QPushButton(centralwidget);
        pushButton->setObjectName(QString::fromUtf8("pushButton"));
        pushButton->setGeometry(QRect(480, 330, 161, 27));
        verticalLayoutWidget = new QWidget(centralwidget);
        verticalLayoutWidget->setObjectName(QString::fromUtf8("verticalLayoutWidget"));
        verticalLayoutWidget->setGeometry(QRect(20, 40, 621, 281));
        verticalLayout = new QVBoxLayout(verticalLayoutWidget);
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        verticalLayout->setContentsMargins(0, 0, 0, 0);
        horizontalLayout_4 = new QHBoxLayout();
        horizontalLayout_4->setObjectName(QString::fromUtf8("horizontalLayout_4"));
        label_4 = new QLabel(verticalLayoutWidget);
        label_4->setObjectName(QString::fromUtf8("label_4"));

        horizontalLayout_4->addWidget(label_4);

        LE_x = new QLineEdit(verticalLayoutWidget);
        LE_x->setObjectName(QString::fromUtf8("LE_x"));
        LE_x->setMaximumSize(QSize(120, 16777215));

        horizontalLayout_4->addWidget(LE_x);

        HS_x = new QSlider(verticalLayoutWidget);
        HS_x->setObjectName(QString::fromUtf8("HS_x"));
        HS_x->setOrientation(Qt::Horizontal);

        horizontalLayout_4->addWidget(HS_x);


        verticalLayout->addLayout(horizontalLayout_4);

        horizontalLayout_5 = new QHBoxLayout();
        horizontalLayout_5->setObjectName(QString::fromUtf8("horizontalLayout_5"));
        label_5 = new QLabel(verticalLayoutWidget);
        label_5->setObjectName(QString::fromUtf8("label_5"));

        horizontalLayout_5->addWidget(label_5);

        LE_y = new QLineEdit(verticalLayoutWidget);
        LE_y->setObjectName(QString::fromUtf8("LE_y"));
        LE_y->setMaximumSize(QSize(120, 16777215));

        horizontalLayout_5->addWidget(LE_y);

        HS_y = new QSlider(verticalLayoutWidget);
        HS_y->setObjectName(QString::fromUtf8("HS_y"));
        HS_y->setOrientation(Qt::Horizontal);

        horizontalLayout_5->addWidget(HS_y);


        verticalLayout->addLayout(horizontalLayout_5);

        horizontalLayout_3 = new QHBoxLayout();
        horizontalLayout_3->setObjectName(QString::fromUtf8("horizontalLayout_3"));
        label_3 = new QLabel(verticalLayoutWidget);
        label_3->setObjectName(QString::fromUtf8("label_3"));

        horizontalLayout_3->addWidget(label_3);

        LE_z = new QLineEdit(verticalLayoutWidget);
        LE_z->setObjectName(QString::fromUtf8("LE_z"));
        LE_z->setMaximumSize(QSize(120, 16777215));

        horizontalLayout_3->addWidget(LE_z);

        HS_z = new QSlider(verticalLayoutWidget);
        HS_z->setObjectName(QString::fromUtf8("HS_z"));
        HS_z->setOrientation(Qt::Horizontal);

        horizontalLayout_3->addWidget(HS_z);


        verticalLayout->addLayout(horizontalLayout_3);

        horizontalLayout_6 = new QHBoxLayout();
        horizontalLayout_6->setObjectName(QString::fromUtf8("horizontalLayout_6"));
        label_6 = new QLabel(verticalLayoutWidget);
        label_6->setObjectName(QString::fromUtf8("label_6"));

        horizontalLayout_6->addWidget(label_6);

        LE_yaw = new QLineEdit(verticalLayoutWidget);
        LE_yaw->setObjectName(QString::fromUtf8("LE_yaw"));
        LE_yaw->setMaximumSize(QSize(120, 16777215));

        horizontalLayout_6->addWidget(LE_yaw);

        HS_yaw = new QSlider(verticalLayoutWidget);
        HS_yaw->setObjectName(QString::fromUtf8("HS_yaw"));
        HS_yaw->setOrientation(Qt::Horizontal);

        horizontalLayout_6->addWidget(HS_yaw);


        verticalLayout->addLayout(horizontalLayout_6);

        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
        label = new QLabel(verticalLayoutWidget);
        label->setObjectName(QString::fromUtf8("label"));

        horizontalLayout->addWidget(label);

        LE_pitch = new QLineEdit(verticalLayoutWidget);
        LE_pitch->setObjectName(QString::fromUtf8("LE_pitch"));
        LE_pitch->setMaximumSize(QSize(120, 16777215));

        horizontalLayout->addWidget(LE_pitch);

        HS_pitch = new QSlider(verticalLayoutWidget);
        HS_pitch->setObjectName(QString::fromUtf8("HS_pitch"));
        HS_pitch->setOrientation(Qt::Horizontal);

        horizontalLayout->addWidget(HS_pitch);


        verticalLayout->addLayout(horizontalLayout);

        horizontalLayout_2 = new QHBoxLayout();
        horizontalLayout_2->setObjectName(QString::fromUtf8("horizontalLayout_2"));
        label_2 = new QLabel(verticalLayoutWidget);
        label_2->setObjectName(QString::fromUtf8("label_2"));

        horizontalLayout_2->addWidget(label_2);

        LE_roll = new QLineEdit(verticalLayoutWidget);
        LE_roll->setObjectName(QString::fromUtf8("LE_roll"));
        LE_roll->setMaximumSize(QSize(120, 16777215));

        horizontalLayout_2->addWidget(LE_roll);

        HS_roll = new QSlider(verticalLayoutWidget);
        HS_roll->setObjectName(QString::fromUtf8("HS_roll"));
        HS_roll->setOrientation(Qt::Horizontal);

        horizontalLayout_2->addWidget(HS_roll);


        verticalLayout->addLayout(horizontalLayout_2);

        PB_MV = new QPushButton(centralwidget);
        PB_MV->setObjectName(QString::fromUtf8("PB_MV"));
        PB_MV->setGeometry(QRect(20, 330, 161, 27));
        MainWindow->setCentralWidget(centralwidget);
        statusbar = new QStatusBar(MainWindow);
        statusbar->setObjectName(QString::fromUtf8("statusbar"));
        MainWindow->setStatusBar(statusbar);

        retranslateUi(MainWindow);

        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QCoreApplication::translate("MainWindow", "MainWindow", nullptr));
        pushButton->setText(QCoreApplication::translate("MainWindow", "PushButton", nullptr));
        label_4->setText(QCoreApplication::translate("MainWindow", "x", nullptr));
        label_5->setText(QCoreApplication::translate("MainWindow", "y", nullptr));
        label_3->setText(QCoreApplication::translate("MainWindow", "z", nullptr));
        label_6->setText(QCoreApplication::translate("MainWindow", "yaw", nullptr));
        label->setText(QCoreApplication::translate("MainWindow", "pitch", nullptr));
        label_2->setText(QCoreApplication::translate("MainWindow", "roll", nullptr));
        PB_MV->setText(QCoreApplication::translate("MainWindow", "MOVE", nullptr));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // TEST1_H

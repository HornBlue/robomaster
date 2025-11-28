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
    QLineEdit *lineEdit_6;
    QSlider *horizontalSlider_6;
    QHBoxLayout *horizontalLayout;
    QLabel *label;
    QLineEdit *lineEdit;
    QSlider *horizontalSlider;
    QHBoxLayout *horizontalLayout_2;
    QLabel *label_2;
    QLineEdit *lineEdit_2;
    QSlider *horizontalSlider_2;
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
        pushButton->setGeometry(QRect(483, 330, 161, 27));
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

        lineEdit_6 = new QLineEdit(verticalLayoutWidget);
        lineEdit_6->setObjectName(QString::fromUtf8("lineEdit_6"));
        lineEdit_6->setMaximumSize(QSize(120, 16777215));

        horizontalLayout_6->addWidget(lineEdit_6);

        horizontalSlider_6 = new QSlider(verticalLayoutWidget);
        horizontalSlider_6->setObjectName(QString::fromUtf8("horizontalSlider_6"));
        horizontalSlider_6->setOrientation(Qt::Horizontal);

        horizontalLayout_6->addWidget(horizontalSlider_6);


        verticalLayout->addLayout(horizontalLayout_6);

        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
        label = new QLabel(verticalLayoutWidget);
        label->setObjectName(QString::fromUtf8("label"));

        horizontalLayout->addWidget(label);

        lineEdit = new QLineEdit(verticalLayoutWidget);
        lineEdit->setObjectName(QString::fromUtf8("lineEdit"));
        lineEdit->setMaximumSize(QSize(120, 16777215));

        horizontalLayout->addWidget(lineEdit);

        horizontalSlider = new QSlider(verticalLayoutWidget);
        horizontalSlider->setObjectName(QString::fromUtf8("horizontalSlider"));
        horizontalSlider->setOrientation(Qt::Horizontal);

        horizontalLayout->addWidget(horizontalSlider);


        verticalLayout->addLayout(horizontalLayout);

        horizontalLayout_2 = new QHBoxLayout();
        horizontalLayout_2->setObjectName(QString::fromUtf8("horizontalLayout_2"));
        label_2 = new QLabel(verticalLayoutWidget);
        label_2->setObjectName(QString::fromUtf8("label_2"));

        horizontalLayout_2->addWidget(label_2);

        lineEdit_2 = new QLineEdit(verticalLayoutWidget);
        lineEdit_2->setObjectName(QString::fromUtf8("lineEdit_2"));
        lineEdit_2->setMaximumSize(QSize(120, 16777215));

        horizontalLayout_2->addWidget(lineEdit_2);

        horizontalSlider_2 = new QSlider(verticalLayoutWidget);
        horizontalSlider_2->setObjectName(QString::fromUtf8("horizontalSlider_2"));
        horizontalSlider_2->setOrientation(Qt::Horizontal);

        horizontalLayout_2->addWidget(horizontalSlider_2);


        verticalLayout->addLayout(horizontalLayout_2);

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
        label_6->setText(QCoreApplication::translate("MainWindow", "TextLabel", nullptr));
        label->setText(QCoreApplication::translate("MainWindow", "TextLabel", nullptr));
        label_2->setText(QCoreApplication::translate("MainWindow", "TextLabel", nullptr));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // TEST1_H

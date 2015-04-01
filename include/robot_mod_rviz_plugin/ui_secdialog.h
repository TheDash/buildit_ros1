/********************************************************************************
** Form generated from reading UI file 'secdialog.ui'
**
** Created: Fri Jan 9 13:24:33 2015
**      by: Qt User Interface Compiler version 4.8.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_SECDIALOG_H
#define UI_SECDIALOG_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QCheckBox>
#include <QtGui/QComboBox>
#include <QtGui/QDialog>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QPushButton>

QT_BEGIN_NAMESPACE

class Ui_SecDialog
{
public:
    QPushButton *addbutton2;
    QPushButton *exportbutton2;
    QPushButton *viewrobotbutton;

    QComboBox *location1combo;
    QLabel *label;
    QLabel *label_2;
    QLabel *label_3;
    QComboBox *location2combo;
    QLabel *label_4;
    QComboBox *location3combo;
    QLabel *label_5;
    QComboBox *location4combo;
    QLabel *label_6;
    QComboBox *location5combo;
    QLabel *label_7;
    QComboBox *location6combo;
    QLabel *label_8;
    QComboBox *location7combo;
    QCheckBox *location1ptu;
    QCheckBox *location2ptu;
    QCheckBox *location3ptu;
    QCheckBox *location4ptu;
    QCheckBox *location5ptu;
    QCheckBox *location6ptu;
    QCheckBox *location7ptu;

    void setupUi(QDialog *SecDialog)
    {
        if (SecDialog->objectName().isEmpty())
            SecDialog->setObjectName(QString::fromUtf8("SecDialog"));
        SecDialog->resize(421, 447);

        viewrobotbutton = new QPushButton(SecDialog);
        viewrobotbutton->setObjectName(QString::fromUtf8("viewrobotbutton"));
        viewrobotbutton->setGeometry(QRect(70, 410, 99, 27));

        addbutton2 = new QPushButton(SecDialog);
        addbutton2->setObjectName(QString::fromUtf8("addbutton2"));
        addbutton2->setGeometry(QRect(180, 410, 99, 27));
        exportbutton2 = new QPushButton(SecDialog);
        exportbutton2->setObjectName(QString::fromUtf8("exportbutton2"));
        exportbutton2->setGeometry(QRect(290, 410, 99, 27));
        location1combo = new QComboBox(SecDialog);
        location1combo->setObjectName(QString::fromUtf8("location1combo"));
        location1combo->setGeometry(QRect(130, 60, 271, 27));
        location1combo->setEditable(true);
        label = new QLabel(SecDialog);
        label->setObjectName(QString::fromUtf8("label"));
        label->setGeometry(QRect(110, 10, 171, 21));
        QFont font;
        font.setFamily(QString::fromUtf8("Century Schoolbook L"));
        font.setPointSize(14);
        font.setItalic(true);
        label->setFont(font);
        label->setFrameShadow(QFrame::Plain);
        label->setLineWidth(1);
        label->setTextFormat(Qt::AutoText);
        label_2 = new QLabel(SecDialog);
        label_2->setObjectName(QString::fromUtf8("label_2"));
        label_2->setGeometry(QRect(130, 40, 271, 17));
        label_3 = new QLabel(SecDialog);
        label_3->setObjectName(QString::fromUtf8("label_3"));
        label_3->setGeometry(QRect(130, 90, 261, 17));
        location2combo = new QComboBox(SecDialog);
        location2combo->setObjectName(QString::fromUtf8("location2combo"));
        location2combo->setGeometry(QRect(130, 110, 271, 27));
        location2combo->setEditable(true);
        label_4 = new QLabel(SecDialog);
        label_4->setObjectName(QString::fromUtf8("label_4"));
        label_4->setGeometry(QRect(130, 140, 271, 17));
        location3combo = new QComboBox(SecDialog);
        location3combo->setObjectName(QString::fromUtf8("location3combo"));
        location3combo->setGeometry(QRect(130, 160, 271, 27));
        location3combo->setEditable(true);
        label_5 = new QLabel(SecDialog);
        label_5->setObjectName(QString::fromUtf8("label_5"));
        label_5->setGeometry(QRect(130, 190, 121, 17));
        location4combo = new QComboBox(SecDialog);
        location4combo->setObjectName(QString::fromUtf8("location4combo"));
        location4combo->setGeometry(QRect(130, 210, 271, 27));
        location4combo->setEditable(true);
        label_6 = new QLabel(SecDialog);
        label_6->setObjectName(QString::fromUtf8("label_6"));
        label_6->setGeometry(QRect(130, 240, 261, 17));
        location5combo = new QComboBox(SecDialog);
        location5combo->setObjectName(QString::fromUtf8("location5combo"));
        location5combo->setGeometry(QRect(130, 260, 271, 27));
        location5combo->setEditable(true);
        label_7 = new QLabel(SecDialog);
        label_7->setObjectName(QString::fromUtf8("label_7"));
        label_7->setGeometry(QRect(130, 290, 191, 17));
        location6combo = new QComboBox(SecDialog);
        location6combo->setObjectName(QString::fromUtf8("location6combo"));
        location6combo->setGeometry(QRect(130, 310, 271, 27));
        location6combo->setEditable(true);
        label_8 = new QLabel(SecDialog);
        label_8->setObjectName(QString::fromUtf8("label_8"));
        label_8->setGeometry(QRect(130, 340, 251, 17));
        location7combo = new QComboBox(SecDialog);
        location7combo->setObjectName(QString::fromUtf8("location7combo"));
        location7combo->setGeometry(QRect(130, 360, 271, 27));
        location7combo->setEditable(true);
        location1ptu = new QCheckBox(SecDialog);
        location1ptu->setObjectName(QString::fromUtf8("location1ptu"));
        location1ptu->setGeometry(QRect(10, 60, 111, 22));
        location1ptu->setLayoutDirection(Qt::RightToLeft);
        location2ptu = new QCheckBox(SecDialog);
        location2ptu->setObjectName(QString::fromUtf8("location2ptu"));
        location2ptu->setGeometry(QRect(10, 110, 111, 22));
        location2ptu->setLayoutDirection(Qt::RightToLeft);
        location3ptu = new QCheckBox(SecDialog);
        location3ptu->setObjectName(QString::fromUtf8("location3ptu"));
        location3ptu->setGeometry(QRect(10, 160, 111, 22));
        location3ptu->setLayoutDirection(Qt::RightToLeft);
        location4ptu = new QCheckBox(SecDialog);
        location4ptu->setObjectName(QString::fromUtf8("location4ptu"));
        location4ptu->setGeometry(QRect(10, 210, 111, 22));
        location4ptu->setLayoutDirection(Qt::RightToLeft);
        location5ptu = new QCheckBox(SecDialog);
        location5ptu->setObjectName(QString::fromUtf8("location5ptu"));
        location5ptu->setGeometry(QRect(10, 260, 111, 22));
        location5ptu->setLayoutDirection(Qt::RightToLeft);
        location6ptu = new QCheckBox(SecDialog);
        location6ptu->setObjectName(QString::fromUtf8("location6ptu"));
        location6ptu->setGeometry(QRect(10, 310, 111, 22));
        location6ptu->setLayoutDirection(Qt::RightToLeft);
        location7ptu = new QCheckBox(SecDialog);
        location7ptu->setObjectName(QString::fromUtf8("location7ptu"));
        location7ptu->setGeometry(QRect(10, 360, 111, 22));
        location7ptu->setLayoutDirection(Qt::RightToLeft);

        retranslateUi(SecDialog);

        QMetaObject::connectSlotsByName(SecDialog);
    } // setupUi

    void retranslateUi(QDialog *SecDialog)
    {
        SecDialog->setWindowTitle(QApplication::translate("SecDialog", "Dialog", 0, QApplication::UnicodeUTF8));
        addbutton2->setText(QApplication::translate("SecDialog", "Add", 0, QApplication::UnicodeUTF8));

        viewrobotbutton->setText(QApplication::translate("SecDialog", "View Robot", 0, QApplication::UnicodeUTF8));

        exportbutton2->setText(QApplication::translate("SecDialog", "Export", 0, QApplication::UnicodeUTF8));
        label->setText(QApplication::translate("SecDialog", "Mounting locations", 0, QApplication::UnicodeUTF8));
        label_2->setText(QApplication::translate("SecDialog", "Top Plate Left", 0, QApplication::UnicodeUTF8));
        label_3->setText(QApplication::translate("SecDialog", "Top Plate Center", 0, QApplication::UnicodeUTF8));
        label_4->setText(QApplication::translate("SecDialog", "Top Plate Right", 0, QApplication::UnicodeUTF8));
        label_5->setText(QApplication::translate("SecDialog", "Sensor Arch Front", 0, QApplication::UnicodeUTF8));
        label_6->setText(QApplication::translate("SecDialog", "Sensor Arch Left", 0, QApplication::UnicodeUTF8));
        label_7->setText(QApplication::translate("SecDialog", "Sensor Arch Center", 0, QApplication::UnicodeUTF8));
        label_8->setText(QApplication::translate("SecDialog", "Sensor Arch Right", 0, QApplication::UnicodeUTF8));
        location1ptu->setText(QApplication::translate("SecDialog", "Pan Tilt Unit", 0, QApplication::UnicodeUTF8));
        location2ptu->setText(QApplication::translate("SecDialog", "Pan Tilt Unit", 0, QApplication::UnicodeUTF8));
        location3ptu->setText(QApplication::translate("SecDialog", "Pan Tilt Unit", 0, QApplication::UnicodeUTF8));
        location4ptu->setText(QApplication::translate("SecDialog", "Pan Tilt Unit", 0, QApplication::UnicodeUTF8));
        location5ptu->setText(QApplication::translate("SecDialog", "Pan Tilt Unit", 0, QApplication::UnicodeUTF8));
        location6ptu->setText(QApplication::translate("SecDialog", "Pan Tilt Unit", 0, QApplication::UnicodeUTF8));
        location7ptu->setText(QApplication::translate("SecDialog", "Pan Tilt Unit", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class SecDialog: public Ui_SecDialog {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_SECDIALOG_H

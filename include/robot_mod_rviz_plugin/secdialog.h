#ifndef SECDIALOG_H
#define SECDIALOG_H

#include <QDialog>

namespace Ui {
class SecDialog;
}

class SecDialog : public QDialog
{
    Q_OBJECT

public:
    explicit SecDialog(QWidget *parent = 0);
    ~SecDialog();

private Q_SLOTS:
    void on_addbutton2_clicked();
    void on_viewrobotbutton_clicked();

private:
    Ui::SecDialog *ui;
};

#endif // SECDIALOG_H

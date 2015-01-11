#include <QWidget>
#include <QPushButton>
#include <QString>
#include <QLabel>
#include <QTableWidget>

class MountPointsTabWidget : public QWidget
{
Q_OBJECT
    public: 
        MountPointsTabWidget(QWidget * parent = 0);
        ~MountPointsTabWidget();

    private:
        QPushButton * load_urdf_button;
        QTableWidget * links_table;
        QTableWidget * selected_links_table;

};

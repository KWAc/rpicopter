#ifndef QMAPS_H
#define QMAPS_H

#include <QtWidgets>
#include <QtWebKitWidgets>


typedef QPair<double, double> PointF;
typedef QList<PointF> RouteF;


class Maps : public QWidget
{
    Q_OBJECT
    
public:
    Maps(QWidget *parent = 0);

public slots:
    void sl_goLeft() {
        moveDxDy(-10, 0);
    }

    void sl_goRight() {
        moveDxDy(+10, 0);
    }

    void sl_goUp() {
        moveDxDy(0, -10);
    }

    void sl_goDown() {
        moveDxDy(0, +10);
    }

    void sl_goTo(float fLat, float fLon) {
        mView->page()->mainFrame()->evaluateJavaScript(QString("map.panTo(new google.maps.LatLng(%1, %2) );").arg(fLat).arg(fLon) );
    }

    void sl_setQuad(float fLat, float fLon, float fYaw) {
        mView->page()->mainFrame()->evaluateJavaScript(QString("setQuad(new google.maps.LatLng(%1, %2), %3);").arg(fLat).arg(fLon).arg(fYaw) );
    }

    void sl_clearRoute() {
        mView->page()->mainFrame()->evaluateJavaScript(QString("clearMarker();") );
    }

    void sl_clearQuad() {
        mView->page()->mainFrame()->evaluateJavaScript(QString("clearQuad();") );
    }

    void sl_submitRoute() {
        qDebug() << "Submit route ..";
        getWaypoints();
    }

    // Returns a list of points: Latitude & Longitude
    RouteF getWaypoints() {
        RouteF lWaypoints;

        QVariant varMarkers = mView->page()->mainFrame()->evaluateJavaScript(QString("getMarkers();") );
        QList<QVariant> allListObj = varMarkers.toList();
        for(int i = 0; i < allListObj.size(); i++) {
            QStringList lPoint = allListObj.at(i).toString().split(':');
            if(lPoint.size() != 2) {
                qDebug() << "Warning: Waypoint not valid!";
                continue;
            }

            double fLat = lPoint.at(0).toDouble();
            double fLon = lPoint.at(1).toDouble();
            PointF waypoint = PointF(fLat, fLon);
            lWaypoints.push_back(waypoint);
            qDebug() << "Added point: " << fLat << ":" << fLon;
        }

        return lWaypoints;
    }

    void sl_moveToDxDy(double) {
        sl_goTo(m_pLat->value(), m_pLon->value() );
    }

private:
    void setupUI();

    void moveDxDy( int dx, int dy ) {
        mView->page()->mainFrame()->evaluateJavaScript( QString("map.panBy(%1, %2);").arg(dx).arg(dy) );
    }

    QDoubleSpinBox *m_pLat;
    QDoubleSpinBox *m_pLon;
    QWebView *mView;
};

#endif // QMAPS_H

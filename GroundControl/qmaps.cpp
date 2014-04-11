#include "qmaps.h"


Maps::Maps(QWidget *parent)
    : QWidget(parent)
{
    setupUI();
}

void Maps::setupUI()
{
    QGridLayout *layout = new QGridLayout( this );

    //
    // Buttons
    //
    QPushButton *btnLeft = new QPushButton( "Left", this );
    QPushButton *btnRight = new QPushButton( "Right", this );
    QPushButton *btnUp = new QPushButton( "Up", this );
    QPushButton *btnDown = new QPushButton( "Down", this );

    QPushButton *btnRemoveRoute = new QPushButton( "Remove route", this );

    QLabel *latitude = new QLabel("Latitude: ");
    QLabel *longitude = new QLabel("Longitude: ");
    m_pLat = new QDoubleSpinBox(this);
    m_pLon = new QDoubleSpinBox(this);

    m_pLat->setRange(-90, 90);
    m_pLon->setRange(-180, 180);

    connect( btnLeft, SIGNAL(clicked()), this, SLOT(sl_goLeft()) );
    connect( btnRight, SIGNAL(clicked()), this, SLOT(sl_goRight()) );
    connect( btnUp, SIGNAL(clicked()), this, SLOT(sl_goUp()) );
    connect( btnDown, SIGNAL(clicked()), this, SLOT(sl_goDown()) );
    connect( btnRemoveRoute, SIGNAL(clicked()), this, SLOT(sl_clearRoute()) );
    connect( m_pLat, SIGNAL(valueChanged(double)), this, SLOT(sl_moveToDxDy(double)) );
    connect( m_pLon, SIGNAL(valueChanged(double)), this, SLOT(sl_moveToDxDy(double)) );

    layout->addWidget( btnLeft, 3, 1 );
    layout->addWidget( btnRight, 3, 3 );
    layout->addWidget( btnUp, 2, 2 );
    layout->addWidget( btnDown, 4, 2 );

    layout->addWidget( latitude, 6, 1, 1, 1 );
    layout->addWidget( longitude, 7, 1, 1, 1 );
    layout->addWidget( m_pLat, 6, 2, 1, 2 );
    layout->addWidget( m_pLon, 7, 2, 1, 2 );

    layout->addWidget( btnRemoveRoute, 8, 1 );

    //
    // Embedded webpage
    //

    mView = new QWebView( this );
    mView->settings()->setAttribute( QWebSettings::JavascriptEnabled, true );
    QString fileName = qApp->applicationDirPath() + "/map.html";
    if( !QFile(fileName).exists() ) {
        qDebug() << "File not found:" << fileName;
    }

    QUrl url = QUrl::fromLocalFile( fileName );
    mView->load( url );

    layout->addWidget( mView, 1, 6, 8, 1 );
    layout->setColumnStretch(6, 1);
}

#include "absframe.h"


CUSTOM_KEY::CUSTOM_KEY() {
    memset(KEY, 0, sizeof(KEY));
}

QAbsFrame::QAbsFrame(QWidget *parent) : QFrame(parent)
{
    m_fWidth = m_fHeight = this->m_fWidth < this->m_fHeight ? this->width() : this->height();
    m_fYaw = 0.f;

    // Set strong focus to enable keyboard events to be received
    setFocusPolicy(Qt::StrongFocus);

    // Init Gamepad values
    m_fStickLX = 0;
    m_fStickLY = 0;
    m_fStickRX = 0;
    m_fStickRY = 0;
    m_fStickL2 = 0;
    m_fStickR2 = 0;

    m_bStickLX = 0;
    m_bStickLY = 0;
    m_bStickRX = 0;
    m_bStickRY = 0;
    m_bStickL2 = 0;
    m_bStickR2 = 0;
    
    m_bButLPressed = 0;
    m_bButDPressed = 0;
    m_bButRPressed = 0;
    m_bButTPressed = 0;

    m_bDeviceArmed = 0;

    m_bGamepadInUse = false;
    m_bGamepadConnected = false;
    initGamepad();
}

void QAbsFrame::initGamepad() {
    sf::Joystick::update();

    // Is joystick #0 connected?
    m_bGamepadConnected = sf::Joystick::isConnected(0);
    
    if(!m_bGamepadConnected) {
        return;
    }
    
    // Does joystick #0 define a X axis?
    m_bStickLX = sf::Joystick::hasAxis(0, sf::Joystick::X);
    m_bStickLY = sf::Joystick::hasAxis(0, sf::Joystick::Y);
    m_bStickRX = sf::Joystick::hasAxis(0, sf::Joystick::Z);
    m_bStickRY = sf::Joystick::hasAxis(0, sf::Joystick::R);
    m_bStickL2 = sf::Joystick::hasAxis(0, sf::Joystick::U);
    m_bStickR2 = sf::Joystick::hasAxis(0, sf::Joystick::V);

    m_tJoystick.start(5);
    QObject::connect(&m_tJoystick, SIGNAL(timeout() ), this, SLOT(updateGamepad() ) );
}

void QAbsFrame::updateGamepad() {
    sf::Joystick::update();

    float fStickLX = 0;
    float fStickLY = 0;
    float fStickRX = 0;
    float fStickRY = 0;
    float fStickL2 = 0;
    float fStickR2 = 0;
    
    // What's the current position of the analog sticks
    if(m_bStickLX) {
        fStickLX = sf::Joystick::getAxisPosition(0, sf::Joystick::X) / 100;
    }
    if(m_bStickLY) {
        fStickLY = sf::Joystick::getAxisPosition(0, sf::Joystick::Y) / 100;
    }
    if(m_bStickRX) {
        fStickRX = sf::Joystick::getAxisPosition(0, sf::Joystick::Z) / 100;
    }
    if(m_bStickRY) {
        fStickRY = sf::Joystick::getAxisPosition(0, sf::Joystick::R) / 100;
    }
    // R2 and L2
    if(m_bStickL2) {
        fStickL2 = sf::Joystick::getAxisPosition(0, sf::Joystick::U) + 100;
        fStickL2 /= 200;
    }
    if(m_bStickR2) {
        fStickR2 = sf::Joystick::getAxisPosition(0, sf::Joystick::V) + 100;
        fStickR2 /= 200;
    }

    if(m_fStickLX != 0 || m_fStickLY != 0 || m_fStickRX != 0 || m_fStickRY != 0 || m_fStickL2 != fStickL2 || m_fStickR2 != fStickR2) {
        m_bGamepadInUse = true;
    } else {
        m_bGamepadInUse = false;
    }

    m_fStickLX = fStickLX;
    m_fStickLY = fStickLY;
    m_fStickRX = fStickRX;
    m_fStickRY = fStickRY;
    m_fStickL2 = fStickL2;
    m_fStickR2 = fStickR2;

    m_bButLPressed = isButtonPressed(0, 0);
    m_bButDPressed = isButtonPressed(0, 1);
    m_bButRPressed = isButtonPressed(0, 2);
    m_bButTPressed = isButtonPressed(0, 3);

    //qDebug() << "m_bButDPressed pressed: " << m_bButDPressed;
    static bool bArmToggle = false;
    if(m_bButDPressed && !bArmToggle) {
        m_bDeviceArmed = m_bDeviceArmed ? false : true;
        bArmToggle = true;
        qDebug() << "Device armed?: " << m_bDeviceArmed;
    }
    if(bArmToggle && !m_bButDPressed) {
        bArmToggle = false;
    }
}

void QAbsFrame::setYaw(float fVal) {
    m_fYaw = fVal;
    QWidget::update();
}

void QAbsFrame::paintEvent(QPaintEvent *pEvent) {
    Q_UNUSED(pEvent);
    QPainter painter(this);

    QPen redPM(Qt::red);

    QPen grayPM(Qt::darkGray);
    grayPM.setWidthF(0);

    QPen blackPM(Qt::black);
    QPen blackPF(Qt::black);
    blackPM.setWidth(2);
    QBrush blackB(Qt::black);
    painter.setRenderHint(QPainter::Antialiasing, true);

    if(!this->isEnabled() ) {
        float centerW   = m_fWidth / 2;
        float centerH   = m_fWidth / 2;

        QFont serifFont("Times", 16, QFont::Bold);
        QRectF rect(-centerW/2.0, -centerH, centerW, centerH);

        painter.translate(m_fWidth/2, m_fHeight/2);
        painter.setFont(serifFont);
        painter.setPen(blackPM);
        painter.drawText(rect, Qt::AlignCenter, tr("No connection.") );
        painter.resetTransform();

        redPM = QPen(Qt::gray);
        grayPM = QPen(Qt::gray);
        blackPF = QPen(Qt::gray);
        blackPM = QPen(Qt::gray);
        blackB = QBrush(Qt::gray);

    }
    // Center plate black
    painter.setBrush(blackB);
    painter.setPen(blackPM);
    float diameter  = m_fWidth / 4;
    float centerW   = m_fWidth / 8;
    float centerH   = m_fWidth / 5.5;

    painter.translate(m_fWidth/2, m_fHeight/2);
    painter.rotate(m_fYaw);
    painter.drawEllipse(QRectF(-centerW/2.0, -centerH/2.0, centerW, centerH));

    // Rotor bottom right
    painter.resetTransform();
    painter.translate(m_fWidth/2, m_fHeight/2);
    painter.rotate(m_fYaw);
    painter.translate(m_fWidth/4, m_fHeight/4);
    painter.setBrush(blackB);
    painter.drawEllipse(QRectF(-diameter/8.0, -diameter/8.0, diameter/4, diameter/4));
    painter.setBrush(Qt::NoBrush);
    painter.drawEllipse(QRectF(-diameter/2.0, -diameter/2.0, diameter, diameter));

    // Rotor top left
    painter.resetTransform();
    painter.translate(m_fWidth/2, m_fHeight/2);
    painter.rotate(m_fYaw);
    painter.translate(-m_fWidth/4, -m_fHeight/4);
    painter.setBrush(blackB);
    painter.drawEllipse(QRectF(-diameter/8.0, -diameter/8.0, diameter/4, diameter/4));
    painter.setBrush(Qt::NoBrush);
    painter.drawEllipse(QRectF(-diameter/2.0, -diameter/2.0, diameter, diameter));

    // Rotor bottom left
    painter.resetTransform();
    painter.translate(m_fWidth/2, m_fHeight/2);
    painter.rotate(m_fYaw);
    painter.translate(-m_fWidth/4, m_fHeight/4);
    painter.setBrush(blackB);
    painter.drawEllipse(QRectF(-diameter/8.0, -diameter/8.0, diameter/4, diameter/4));
    painter.setBrush(Qt::NoBrush);
    painter.drawEllipse(QRectF(-diameter/2.0, -diameter/2.0, diameter, diameter));

    // Rotor top right
    painter.resetTransform();
    painter.translate(m_fWidth/2, m_fHeight/2);
    painter.rotate(m_fYaw);
    painter.translate(m_fWidth/4, -m_fHeight/4);
    painter.setBrush(blackB);
    painter.drawEllipse(QRectF(-diameter/8.0, -diameter/8.0, diameter/4, diameter/4));
    painter.setBrush(Qt::NoBrush);
    painter.drawEllipse(QRectF(-diameter/2.0, -diameter/2.0, diameter, diameter));


    // Now draw the other frame parts
    QLineF bl_fr(-m_fWidth/4, m_fHeight/4, m_fWidth/4, -m_fHeight/4);
    QLineF br_fl(m_fWidth/4, m_fHeight/4, -m_fWidth/4, -m_fHeight/4);

    blackPF.setWidth(m_fWidth < m_fHeight ? m_fWidth/48 : m_fHeight/48);
    painter.setPen(blackPF);

    painter.resetTransform();
    painter.translate(m_fWidth/2, m_fHeight/2);
    painter.rotate(m_fYaw);

    painter.drawLine(bl_fr);
    painter.drawLine(br_fl);

    if(this->isEnabled() ) {
        // Highlighting active motors ..
        QPen redHighlightP(QColor(255,0,0, 45));
        redHighlightP.setWidth(16);
        QBrush redHighlightB(QColor(255,0,0, 25));
        painter.setPen(redHighlightP);
        painter.setBrush(redHighlightB);
        // bottom in red
        if(m_customKeyStatus[Qt::Key_W] == true) {
            // Rotor bottom left
            painter.resetTransform();
            painter.translate(m_fWidth/2, m_fHeight/2);
            painter.rotate(m_fYaw);
            painter.translate(-m_fWidth/4, m_fHeight/4);
            painter.drawEllipse(QRectF(-diameter/2.0, -diameter/2.0, diameter, diameter));

            // Rotor bottom right
            painter.resetTransform();
            painter.translate(m_fWidth/2, m_fHeight/2);
            painter.rotate(m_fYaw);
            painter.translate(m_fWidth/4, m_fHeight/4);
            painter.drawEllipse(QRectF(-diameter/2.0, -diameter/2.0, diameter, diameter));
        }
        // top in red
        if(m_customKeyStatus[Qt::Key_S] == true) {
            // Rotor top left
            painter.resetTransform();
            painter.translate(m_fWidth/2, m_fHeight/2);
            painter.rotate(m_fYaw);
            painter.translate(-m_fWidth/4, -m_fHeight/4);
            painter.drawEllipse(QRectF(-diameter/2.0, -diameter/2.0, diameter, diameter));

            // Rotor top right
            painter.resetTransform();
            painter.translate(m_fWidth/2, m_fHeight/2);
            painter.rotate(m_fYaw);
            painter.translate(m_fWidth/4, -m_fHeight/4);
            painter.drawEllipse(QRectF(-diameter/2.0, -diameter/2.0, diameter, diameter));
        }

        // right in red
        if(m_customKeyStatus[Qt::Key_A] == true) {
            // Rotor bottom right
            painter.resetTransform();
            painter.translate(m_fWidth/2, m_fHeight/2);
            painter.rotate(m_fYaw);
            painter.translate(m_fWidth/4, m_fHeight/4);
            painter.drawEllipse(QRectF(-diameter/2.0, -diameter/2.0, diameter, diameter));

            // Rotor top right
            painter.resetTransform();
            painter.translate(m_fWidth/2, m_fHeight/2);
            painter.rotate(m_fYaw);
            painter.translate(m_fWidth/4, -m_fHeight/4);
            painter.drawEllipse(QRectF(-diameter/2.0, -diameter/2.0, diameter, diameter));
        }

        // left in red
        if(m_customKeyStatus[Qt::Key_D] == true) {
            // Rotor top left
            painter.resetTransform();
            painter.translate(m_fWidth/2, m_fHeight/2);
            painter.rotate(m_fYaw);
            painter.translate(-m_fWidth/4, -m_fHeight/4);
            painter.drawEllipse(QRectF(-diameter/2.0, -diameter/2.0, diameter, diameter));

            // Rotor bottom left
            painter.resetTransform();
            painter.translate(m_fWidth/2, m_fHeight/2);
            painter.rotate(m_fYaw);
            painter.translate(-m_fWidth/4, m_fHeight/4);
            painter.drawEllipse(QRectF(-diameter/2.0, -diameter/2.0, diameter, diameter));
        }


        // bottom left and top right in red
        if(m_customKeyStatus[Qt::Key_Q] == true) {
            // Rotor bottom left
            painter.resetTransform();
            painter.translate(m_fWidth/2, m_fHeight/2);
            painter.rotate(m_fYaw);
            painter.translate(-m_fWidth/4, m_fHeight/4);
            painter.drawEllipse(QRectF(-diameter/2.0, -diameter/2.0, diameter, diameter));

            // Rotor top right
            painter.resetTransform();
            painter.translate(m_fWidth/2, m_fHeight/2);
            painter.rotate(m_fYaw);
            painter.translate(m_fWidth/4, -m_fHeight/4);
            painter.drawEllipse(QRectF(-diameter/2.0, -diameter/2.0, diameter, diameter));
        }

        // bottom right and top left in red
        if(m_customKeyStatus[Qt::Key_E] == true) {
            // Rotor bottom right
            painter.resetTransform();
            painter.translate(m_fWidth/2, m_fHeight/2);
            painter.rotate(m_fYaw);
            painter.translate(m_fWidth/4, m_fHeight/4);
            painter.drawEllipse(QRectF(-diameter/2.0, -diameter/2.0, diameter, diameter));

            // Rotor top left
            painter.resetTransform();
            painter.translate(m_fWidth/2, m_fHeight/2);
            painter.rotate(m_fYaw);
            painter.translate(-m_fWidth/4, -m_fHeight/4);
            painter.drawEllipse(QRectF(-diameter/2.0, -diameter/2.0, diameter, diameter));
        }
    }

    // Draw Coordinate system
    painter.setRenderHint(QPainter::Antialiasing, false);
    painter.setPen(grayPM);
    painter.setBrush(Qt::NoBrush);

    float fWAx = 0.1 * m_fWidth;
    float fHAx = 0.1 * m_fHeight;

    float fWC = 0.025 * m_fWidth;
    float fHC = 0.025 * m_fHeight;

    painter.resetTransform();
    painter.translate(m_fWidth/2, m_fHeight/2);

    painter.drawEllipse(QRectF(-(m_fWidth/2-fWC/2), -(m_fHeight/2-fHC/2), m_fWidth-fWC, m_fHeight-fHC));
    QLineF lineC90 ( m_fWidth/2 -fWC/2 - 5, 0, m_fWidth/2 -fWC/2 + 5, 0 );
    QLineF lineC270( -(m_fWidth/2 -fWC/2 - 5), 0, -(m_fWidth/2 -fWC/2 + 5), 0 );
    QLineF lineC180( 0, m_fHeight/2 -fHC/2 - 5, 0, m_fHeight/2 -fHC/2 + 5 );
    QLineF lineC360( 0, -(m_fHeight/2 -fHC/2 - 5), 0, -(m_fHeight/2 -fHC/2 + 5) );

    painter.drawLine(lineC90);
    painter.drawLine(lineC180);
    painter.drawLine(lineC270);
    painter.drawLine(lineC360);

    // X and Y axis
    QLineF lineAxisY( 0, -m_fHeight/2 + fHAx, 0, 0 );
    QLineF lineAxisX( 0, 0, m_fWidth/2 - fWAx, 0 );

    painter.rotate(m_fYaw);
    painter.drawLine(lineAxisY);
    painter.setPen(redPM);
    painter.drawLine(lineAxisX);
}

void QAbsFrame::keyPressEvent ( QKeyEvent * event ) {
    if(event->isAutoRepeat() )
        return;

    int key = event->key();
    int i = CUSTOM_KEY::mapCustomKeyIndex(key);
    switch (key) {
        case Qt::Key_Backspace:
        m_customKeyStatus[i] = true;
        //qDebug() << "Backspace";
        break;

        case Qt::Key_A:
        m_customKeyStatus[i] = true;
        //qDebug() << "A";
        break;
        case Qt::Key_D:
        m_customKeyStatus[i] = true;
        //qDebug() << "D";
        break;

        case Qt::Key_W:
        m_customKeyStatus[i] = true;
        //qDebug() << "W";
        break;
        case Qt::Key_S:
        m_customKeyStatus[i] = true;
        //qDebug() << "S";
        break;

        case Qt::Key_Q:
        m_customKeyStatus[i] = true;
        //qDebug() << "Q";
        break;
        case Qt::Key_E:
        m_customKeyStatus[i] = true;
        //qDebug() << "E";
        break;

        case Qt::Key_C:
        m_customKeyStatus[i] = true;
        //qDebug() << "C";
        break;

        case Qt::Key_H:
        m_customKeyStatus[i] = true;
        //qDebug() << "H";
        break;

        case Qt::Key_Up:
        m_customKeyStatus[i] = true;
        //qDebug() << "Up";
        break;
        case Qt::Key_Down:
        m_customKeyStatus[i] = true;
        //qDebug() << "Down";
        break;

        case Qt::Key_F1:
        m_customKeyStatus[i] = true;
        //qDebug() << "F1";
        break;
        case Qt::Key_F2:
        m_customKeyStatus[i] = true;
        //qDebug() << "F2";
        break;
        case Qt::Key_F3:
        m_customKeyStatus[i] = true;
        //qDebug() << "F3";
        break;
        case Qt::Key_F4:
        m_customKeyStatus[i] = true;
        //qDebug() << "F4";
        break;

        case Qt::Key_8:
        m_customKeyStatus[i] = true;
        //qDebug() << "F8";
        break;
        case Qt::Key_2:
        m_customKeyStatus[i] = true;
        //qDebug() << "F2";
        break;
        case Qt::Key_4:
        m_customKeyStatus[i] = true;
        //qDebug() << "F4";
        break;
        case Qt::Key_6:
        m_customKeyStatus[i] = true;
        //qDebug() << "F6";
        break;
    }
    QWidget::update();
}

void QAbsFrame::keyReleaseEvent ( QKeyEvent * event ) {
    if(event->isAutoRepeat() )
        return;

    int key = event->key();
    int i = CUSTOM_KEY::mapCustomKeyIndex(key);
    switch (key) {
        case Qt::Key_Backspace:
        m_customKeyStatus[i] = false;
        break;

        case Qt::Key_A:
        m_customKeyStatus[i] = false;
        break;
        case Qt::Key_D:
        m_customKeyStatus[i] = false;
        break;

        case Qt::Key_W:
        m_customKeyStatus[i] = false;
        break;
        case Qt::Key_S:
        m_customKeyStatus[i] = false;
        break;

        case Qt::Key_Q:
        m_customKeyStatus[i] = false;
        break;
        case Qt::Key_E:
        m_customKeyStatus[i] = false;
        break;

        case Qt::Key_C:
        m_customKeyStatus[i] = false;
        break;

        case Qt::Key_H:
        m_customKeyStatus[i] = false;
        break;

        case Qt::Key_Up:
        m_customKeyStatus[i] = false;
        break;
        case Qt::Key_Down:
        m_customKeyStatus[i] = false;
        break;

        case Qt::Key_F1:
        m_customKeyStatus[i] = false;
        break;
        case Qt::Key_F2:
        m_customKeyStatus[i] = false;
        break;
        case Qt::Key_F3:
        m_customKeyStatus[i] = false;
        break;
        case Qt::Key_F4:
        m_customKeyStatus[i] = false;
        break;

        case Qt::Key_8:
        m_customKeyStatus[i] = false;
        break;
        case Qt::Key_2:
        m_customKeyStatus[i] = false;
        break;
        case Qt::Key_4:
        m_customKeyStatus[i] = false;
        break;
        case Qt::Key_6:
        m_customKeyStatus[i] = false;
        break;
    }
    QWidget::update();
}

void QAbsFrame::resizeEvent( QResizeEvent * event ) {
    m_fWidth = m_fHeight = this->m_fWidth < this->m_fHeight ? event->size().width() : event->size().height();
}

#ifndef QABSFRAME_H
#define QABSFRAME_H

#include <math.h>
#include <QtWidgets>
#include <SFML/Window.hpp>


struct CUSTOM_KEY {
    CUSTOM_KEY();

    bool KEY[512];

    bool& operator[](int const& index) {
        return KEY[index];
    }
    const bool& operator[](int const& index) const {
        return KEY[index];
    }

    static int mapCustomKeyIndex(const int key);
};

class QAbsFrame : public QFrame, public sf::Joystick {
    Q_OBJECT

    private:
        float m_fYaw;
        float m_fWidth;
        float m_fHeight;

        QTimer m_tJoystick;

        void initGamepad();

    protected:
        CUSTOM_KEY m_customKeyStatus;

        void paintEvent ( QPaintEvent * );
        void keyPressEvent ( QKeyEvent * );
        void keyReleaseEvent ( QKeyEvent * );
        void resizeEvent ( QResizeEvent * );

        // Just store the gamepad data into protected variables
        bool m_bStickLX;
        bool m_bStickLY;
        bool m_bStickRX;
        bool m_bStickRY;
        bool m_bStickL2;
        bool m_bStickR2;
        
        bool m_bButLPressed;
        bool m_bButDPressed;
        bool m_bButRPressed;
        bool m_bButTPressed;

        float m_fStickLX;
        float m_fStickLY;
        float m_fStickRX;
        float m_fStickRY;
        float m_fStickL2;
        float m_fStickR2;

        bool m_bGamepadInUse;
        bool m_bDeviceArmed;
        bool m_bGamepadConnected;

    protected slots:
        void updateGamepad();

    public:
        QAbsFrame(QWidget *parent = NULL);

        void setYaw(float fVal);
    };

#endif // QABSFRAME_H

#ifndef QABSFRAME_H
#define QABSFRAME_H

#include <math.h>
#include <QtWidgets>


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

class QAbsFrame : public QFrame {
    Q_OBJECT

    private:
        float m_fYaw;
        float m_fWidth;
        float m_fHeight;

    protected:
        CUSTOM_KEY m_customKeyStatus;

        void paintEvent ( QPaintEvent * );
        void keyPressEvent ( QKeyEvent * );
        void keyReleaseEvent ( QKeyEvent * );
        void resizeEvent ( QResizeEvent * );

    public:
        QAbsFrame(QWidget *parent = NULL);

        void setYaw(float fVal);
    };

#endif // QABSFRAME_H

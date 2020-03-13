/****************************************************************************
** Meta object code from reading C++ file 'mygraphicwindowqt.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.7.1)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "mygraphicwindowqt.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'mygraphicwindowqt.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.7.1. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
struct qt_meta_stringdata_MyGraphicWindowQt_t {
    QByteArrayData data[6];
    char stringdata0[98];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_MyGraphicWindowQt_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_MyGraphicWindowQt_t qt_meta_stringdata_MyGraphicWindowQt = {
    {
QT_MOC_LITERAL(0, 0, 17), // "MyGraphicWindowQt"
QT_MOC_LITERAL(1, 18, 28), // "moveOpenglContextToNewThread"
QT_MOC_LITERAL(2, 47, 0), // ""
QT_MOC_LITERAL(3, 48, 8), // "QThread*"
QT_MOC_LITERAL(4, 57, 9), // "newThread"
QT_MOC_LITERAL(5, 67, 30) // "onMoveOpenglContextToNewThread"

    },
    "MyGraphicWindowQt\0moveOpenglContextToNewThread\0"
    "\0QThread*\0newThread\0onMoveOpenglContextToNewThread"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_MyGraphicWindowQt[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
       2,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       1,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    1,   24,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
       5,    1,   27,    2, 0x0a /* Public */,

 // signals: parameters
    QMetaType::Void, 0x80000000 | 3,    4,

 // slots: parameters
    QMetaType::Void, 0x80000000 | 3,    4,

       0        // eod
};

void MyGraphicWindowQt::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        MyGraphicWindowQt *_t = static_cast<MyGraphicWindowQt *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->moveOpenglContextToNewThread((*reinterpret_cast< QThread*(*)>(_a[1]))); break;
        case 1: _t->onMoveOpenglContextToNewThread((*reinterpret_cast< QThread*(*)>(_a[1]))); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        void **func = reinterpret_cast<void **>(_a[1]);
        {
            typedef void (MyGraphicWindowQt::*_t)(QThread * );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&MyGraphicWindowQt::moveOpenglContextToNewThread)) {
                *result = 0;
                return;
            }
        }
    }
}

const QMetaObject MyGraphicWindowQt::staticMetaObject = {
    { &QObject::staticMetaObject, qt_meta_stringdata_MyGraphicWindowQt.data,
      qt_meta_data_MyGraphicWindowQt,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *MyGraphicWindowQt::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *MyGraphicWindowQt::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_MyGraphicWindowQt.stringdata0))
        return static_cast<void*>(const_cast< MyGraphicWindowQt*>(this));
    if (!strcmp(_clname, "osgViewer::GraphicsWindow"))
        return static_cast< osgViewer::GraphicsWindow*>(const_cast< MyGraphicWindowQt*>(this));
    return QObject::qt_metacast(_clname);
}

int MyGraphicWindowQt::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QObject::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 2)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 2;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 2)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 2;
    }
    return _id;
}

// SIGNAL 0
void MyGraphicWindowQt::moveOpenglContextToNewThread(QThread * _t1)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}
QT_END_MOC_NAMESPACE

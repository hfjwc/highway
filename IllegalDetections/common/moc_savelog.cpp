/****************************************************************************
** Meta object code from reading C++ file 'savelog.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.14.0)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include <memory>
#include "savelog.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'savelog.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.14.0. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_SaveLog_t {
    QByteArrayData data[19];
    char stringdata0[141];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_SaveLog_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_SaveLog_t qt_meta_stringdata_SaveLog = {
    {
QT_MOC_LITERAL(0, 0, 7), // "SaveLog"
QT_MOC_LITERAL(1, 8, 5), // "start"
QT_MOC_LITERAL(2, 14, 0), // ""
QT_MOC_LITERAL(3, 15, 4), // "stop"
QT_MOC_LITERAL(4, 20, 5), // "clear"
QT_MOC_LITERAL(5, 26, 4), // "save"
QT_MOC_LITERAL(6, 31, 7), // "content"
QT_MOC_LITERAL(7, 39, 9), // "setMaxRow"
QT_MOC_LITERAL(8, 49, 6), // "maxRow"
QT_MOC_LITERAL(9, 56, 10), // "setMaxSize"
QT_MOC_LITERAL(10, 67, 7), // "maxSize"
QT_MOC_LITERAL(11, 75, 7), // "setPath"
QT_MOC_LITERAL(12, 83, 4), // "path"
QT_MOC_LITERAL(13, 88, 7), // "setName"
QT_MOC_LITERAL(14, 96, 4), // "name"
QT_MOC_LITERAL(15, 101, 10), // "setMsgType"
QT_MOC_LITERAL(16, 112, 7), // "MsgType"
QT_MOC_LITERAL(17, 120, 7), // "msgType"
QT_MOC_LITERAL(18, 128, 12) // "onTimerClear"

    },
    "SaveLog\0start\0\0stop\0clear\0save\0content\0"
    "setMaxRow\0maxRow\0setMaxSize\0maxSize\0"
    "setPath\0path\0setName\0name\0setMsgType\0"
    "MsgType\0msgType\0onTimerClear"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_SaveLog[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
      10,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    0,   64,    2, 0x0a /* Public */,
       3,    0,   65,    2, 0x0a /* Public */,
       4,    0,   66,    2, 0x0a /* Public */,
       5,    1,   67,    2, 0x0a /* Public */,
       7,    1,   70,    2, 0x0a /* Public */,
       9,    1,   73,    2, 0x0a /* Public */,
      11,    1,   76,    2, 0x0a /* Public */,
      13,    1,   79,    2, 0x0a /* Public */,
      15,    1,   82,    2, 0x0a /* Public */,
      18,    0,   85,    2, 0x0a /* Public */,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::QString,    6,
    QMetaType::Void, QMetaType::Int,    8,
    QMetaType::Void, QMetaType::Int,   10,
    QMetaType::Void, QMetaType::QString,   12,
    QMetaType::Void, QMetaType::QString,   14,
    QMetaType::Void, 0x80000000 | 16,   17,
    QMetaType::Void,

       0        // eod
};

void SaveLog::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<SaveLog *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->start(); break;
        case 1: _t->stop(); break;
        case 2: _t->clear(); break;
        case 3: _t->save((*reinterpret_cast< const QString(*)>(_a[1]))); break;
        case 4: _t->setMaxRow((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 5: _t->setMaxSize((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 6: _t->setPath((*reinterpret_cast< const QString(*)>(_a[1]))); break;
        case 7: _t->setName((*reinterpret_cast< const QString(*)>(_a[1]))); break;
        case 8: _t->setMsgType((*reinterpret_cast< const MsgType(*)>(_a[1]))); break;
        case 9: _t->onTimerClear(); break;
        default: ;
        }
    }
}

QT_INIT_METAOBJECT const QMetaObject SaveLog::staticMetaObject = { {
    QMetaObject::SuperData::link<QObject::staticMetaObject>(),
    qt_meta_stringdata_SaveLog.data,
    qt_meta_data_SaveLog,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *SaveLog::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *SaveLog::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_SaveLog.stringdata0))
        return static_cast<void*>(this);
    return QObject::qt_metacast(_clname);
}

int SaveLog::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QObject::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 10)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 10;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 10)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 10;
    }
    return _id;
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE

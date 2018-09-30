/****************************************************************************
** Meta object code from reading C++ file 'Scene_item.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.4.2)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "Scene_item.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'Scene_item.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.4.2. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
struct qt_meta_stringdata_CGAL__Three__Scene_item_t {
    QByteArrayData data[25];
    char stringdata[205];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_CGAL__Three__Scene_item_t, stringdata) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_CGAL__Three__Scene_item_t qt_meta_stringdata_CGAL__Three__Scene_item = {
    {
QT_MOC_LITERAL(0, 0, 23), // "CGAL::Three::Scene_item"
QT_MOC_LITERAL(1, 24, 11), // "itemChanged"
QT_MOC_LITERAL(2, 36, 0), // ""
QT_MOC_LITERAL(3, 37, 18), // "aboutToBeDestroyed"
QT_MOC_LITERAL(4, 56, 6), // "redraw"
QT_MOC_LITERAL(5, 63, 8), // "setColor"
QT_MOC_LITERAL(6, 72, 1), // "c"
QT_MOC_LITERAL(7, 74, 11), // "setRbgColor"
QT_MOC_LITERAL(8, 86, 1), // "r"
QT_MOC_LITERAL(9, 88, 1), // "g"
QT_MOC_LITERAL(10, 90, 1), // "b"
QT_MOC_LITERAL(11, 92, 7), // "setName"
QT_MOC_LITERAL(12, 100, 1), // "n"
QT_MOC_LITERAL(13, 102, 10), // "setVisible"
QT_MOC_LITERAL(14, 113, 22), // "itemAboutToBeDestroyed"
QT_MOC_LITERAL(15, 136, 11), // "Scene_item*"
QT_MOC_LITERAL(16, 148, 6), // "select"
QT_MOC_LITERAL(17, 155, 6), // "orig_x"
QT_MOC_LITERAL(18, 162, 6), // "orig_y"
QT_MOC_LITERAL(19, 169, 6), // "orig_z"
QT_MOC_LITERAL(20, 176, 5), // "dir_x"
QT_MOC_LITERAL(21, 182, 5), // "dir_y"
QT_MOC_LITERAL(22, 188, 5), // "dir_z"
QT_MOC_LITERAL(23, 194, 5), // "color"
QT_MOC_LITERAL(24, 200, 4) // "name"

    },
    "CGAL::Three::Scene_item\0itemChanged\0"
    "\0aboutToBeDestroyed\0redraw\0setColor\0"
    "c\0setRbgColor\0r\0g\0b\0setName\0n\0setVisible\0"
    "itemAboutToBeDestroyed\0Scene_item*\0"
    "select\0orig_x\0orig_y\0orig_z\0dir_x\0"
    "dir_y\0dir_z\0color\0name"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_CGAL__Three__Scene_item[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
       9,   14, // methods
       2,   94, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       3,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    0,   59,    2, 0x06 /* Public */,
       3,    0,   60,    2, 0x06 /* Public */,
       4,    0,   61,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
       5,    1,   62,    2, 0x0a /* Public */,
       7,    3,   65,    2, 0x0a /* Public */,
      11,    1,   72,    2, 0x0a /* Public */,
      13,    1,   75,    2, 0x0a /* Public */,
      14,    1,   78,    2, 0x0a /* Public */,
      16,    6,   81,    2, 0x0a /* Public */,

 // signals: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,

 // slots: parameters
    QMetaType::Void, QMetaType::QColor,    6,
    QMetaType::Void, QMetaType::Int, QMetaType::Int, QMetaType::Int,    8,    9,   10,
    QMetaType::Void, QMetaType::QString,   12,
    QMetaType::Void, QMetaType::Bool,   10,
    QMetaType::Void, 0x80000000 | 15,    2,
    QMetaType::Void, QMetaType::Double, QMetaType::Double, QMetaType::Double, QMetaType::Double, QMetaType::Double, QMetaType::Double,   17,   18,   19,   20,   21,   22,

 // properties: name, type, flags
      23, QMetaType::QColor, 0x00095103,
      24, QMetaType::QString, 0x00095103,

 // enums: name, flags, count, data

 // enum data: key, value

       0        // eod
};

void CGAL::Three::Scene_item::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Scene_item *_t = static_cast<Scene_item *>(_o);
        switch (_id) {
        case 0: _t->itemChanged(); break;
        case 1: _t->aboutToBeDestroyed(); break;
        case 2: _t->redraw(); break;
        case 3: _t->setColor((*reinterpret_cast< QColor(*)>(_a[1]))); break;
        case 4: _t->setRbgColor((*reinterpret_cast< int(*)>(_a[1])),(*reinterpret_cast< int(*)>(_a[2])),(*reinterpret_cast< int(*)>(_a[3]))); break;
        case 5: _t->setName((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 6: _t->setVisible((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 7: _t->itemAboutToBeDestroyed((*reinterpret_cast< Scene_item*(*)>(_a[1]))); break;
        case 8: _t->select((*reinterpret_cast< double(*)>(_a[1])),(*reinterpret_cast< double(*)>(_a[2])),(*reinterpret_cast< double(*)>(_a[3])),(*reinterpret_cast< double(*)>(_a[4])),(*reinterpret_cast< double(*)>(_a[5])),(*reinterpret_cast< double(*)>(_a[6]))); break;
        default: ;
        }
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        switch (_id) {
        default: *reinterpret_cast<int*>(_a[0]) = -1; break;
        case 7:
            switch (*reinterpret_cast<int*>(_a[1])) {
            default: *reinterpret_cast<int*>(_a[0]) = -1; break;
            case 0:
                *reinterpret_cast<int*>(_a[0]) = qRegisterMetaType< Scene_item* >(); break;
            }
            break;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        void **func = reinterpret_cast<void **>(_a[1]);
        {
            typedef void (Scene_item::*_t)();
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&Scene_item::itemChanged)) {
                *result = 0;
            }
        }
        {
            typedef void (Scene_item::*_t)();
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&Scene_item::aboutToBeDestroyed)) {
                *result = 1;
            }
        }
        {
            typedef void (Scene_item::*_t)();
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&Scene_item::redraw)) {
                *result = 2;
            }
        }
    }
}

const QMetaObject CGAL::Three::Scene_item::staticMetaObject = {
    { &QObject::staticMetaObject, qt_meta_stringdata_CGAL__Three__Scene_item.data,
      qt_meta_data_CGAL__Three__Scene_item,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *CGAL::Three::Scene_item::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *CGAL::Three::Scene_item::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_CGAL__Three__Scene_item.stringdata))
        return static_cast<void*>(const_cast< Scene_item*>(this));
    return QObject::qt_metacast(_clname);
}

int CGAL::Three::Scene_item::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QObject::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 9)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 9;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 9)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 9;
    }
#ifndef QT_NO_PROPERTIES
      else if (_c == QMetaObject::ReadProperty) {
        void *_v = _a[0];
        switch (_id) {
        case 0: *reinterpret_cast< QColor*>(_v) = color(); break;
        case 1: *reinterpret_cast< QString*>(_v) = name(); break;
        default: break;
        }
        _id -= 2;
    } else if (_c == QMetaObject::WriteProperty) {
        void *_v = _a[0];
        switch (_id) {
        case 0: setColor(*reinterpret_cast< QColor*>(_v)); break;
        case 1: setName(*reinterpret_cast< QString*>(_v)); break;
        default: break;
        }
        _id -= 2;
    } else if (_c == QMetaObject::ResetProperty) {
        _id -= 2;
    } else if (_c == QMetaObject::QueryPropertyDesignable) {
        _id -= 2;
    } else if (_c == QMetaObject::QueryPropertyScriptable) {
        _id -= 2;
    } else if (_c == QMetaObject::QueryPropertyStored) {
        _id -= 2;
    } else if (_c == QMetaObject::QueryPropertyEditable) {
        _id -= 2;
    } else if (_c == QMetaObject::QueryPropertyUser) {
        _id -= 2;
    } else if (_c == QMetaObject::RegisterPropertyMetaType) {
        if (_id < 2)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 2;
    }
#endif // QT_NO_PROPERTIES
    return _id;
}

// SIGNAL 0
void CGAL::Three::Scene_item::itemChanged()
{
    QMetaObject::activate(this, &staticMetaObject, 0, Q_NULLPTR);
}

// SIGNAL 1
void CGAL::Three::Scene_item::aboutToBeDestroyed()
{
    QMetaObject::activate(this, &staticMetaObject, 1, Q_NULLPTR);
}

// SIGNAL 2
void CGAL::Three::Scene_item::redraw()
{
    QMetaObject::activate(this, &staticMetaObject, 2, Q_NULLPTR);
}
QT_END_MOC_NAMESPACE

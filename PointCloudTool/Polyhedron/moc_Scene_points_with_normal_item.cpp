/****************************************************************************
** Meta object code from reading C++ file 'Scene_points_with_normal_item.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.4.2)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "Scene_points_with_normal_item.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'Scene_points_with_normal_item.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.4.2. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
struct qt_meta_stringdata_Scene_points_with_normal_item_t {
    QByteArrayData data[12];
    char stringdata[191];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_Scene_points_with_normal_item_t, stringdata) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_Scene_points_with_normal_item_t qt_meta_stringdata_Scene_points_with_normal_item = {
    {
QT_MOC_LITERAL(0, 0, 29), // "Scene_points_with_normal_item"
QT_MOC_LITERAL(1, 30, 15), // "deleteSelection"
QT_MOC_LITERAL(2, 46, 0), // ""
QT_MOC_LITERAL(3, 47, 15), // "invertSelection"
QT_MOC_LITERAL(4, 63, 9), // "selectAll"
QT_MOC_LITERAL(5, 73, 14), // "resetSelection"
QT_MOC_LITERAL(6, 88, 16), // "selectDuplicates"
QT_MOC_LITERAL(7, 105, 18), // "pointSliderPressed"
QT_MOC_LITERAL(8, 124, 19), // "pointSliderReleased"
QT_MOC_LITERAL(9, 144, 22), // "itemAboutToBeDestroyed"
QT_MOC_LITERAL(10, 167, 11), // "Scene_item*"
QT_MOC_LITERAL(11, 179, 11) // "resetColors"

    },
    "Scene_points_with_normal_item\0"
    "deleteSelection\0\0invertSelection\0"
    "selectAll\0resetSelection\0selectDuplicates\0"
    "pointSliderPressed\0pointSliderReleased\0"
    "itemAboutToBeDestroyed\0Scene_item*\0"
    "resetColors"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_Scene_points_with_normal_item[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
       9,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    0,   59,    2, 0x0a /* Public */,
       3,    0,   60,    2, 0x0a /* Public */,
       4,    0,   61,    2, 0x0a /* Public */,
       5,    0,   62,    2, 0x0a /* Public */,
       6,    0,   63,    2, 0x0a /* Public */,
       7,    0,   64,    2, 0x0a /* Public */,
       8,    0,   65,    2, 0x0a /* Public */,
       9,    1,   66,    2, 0x0a /* Public */,
      11,    0,   69,    2, 0x0a /* Public */,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, 0x80000000 | 10,    2,
    QMetaType::Void,

       0        // eod
};

void Scene_points_with_normal_item::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Scene_points_with_normal_item *_t = static_cast<Scene_points_with_normal_item *>(_o);
        switch (_id) {
        case 0: _t->deleteSelection(); break;
        case 1: _t->invertSelection(); break;
        case 2: _t->selectAll(); break;
        case 3: _t->resetSelection(); break;
        case 4: _t->selectDuplicates(); break;
        case 5: _t->pointSliderPressed(); break;
        case 6: _t->pointSliderReleased(); break;
        case 7: _t->itemAboutToBeDestroyed((*reinterpret_cast< Scene_item*(*)>(_a[1]))); break;
        case 8: _t->resetColors(); break;
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
    }
}

const QMetaObject Scene_points_with_normal_item::staticMetaObject = {
    { &CGAL::Three::Scene_item::staticMetaObject, qt_meta_stringdata_Scene_points_with_normal_item.data,
      qt_meta_data_Scene_points_with_normal_item,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *Scene_points_with_normal_item::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *Scene_points_with_normal_item::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_Scene_points_with_normal_item.stringdata))
        return static_cast<void*>(const_cast< Scene_points_with_normal_item*>(this));
    if (!strcmp(_clname, "CGAL::Three::Scene_item_with_properties"))
        return static_cast< CGAL::Three::Scene_item_with_properties*>(const_cast< Scene_points_with_normal_item*>(this));
    return CGAL::Three::Scene_item::qt_metacast(_clname);
}

int Scene_points_with_normal_item::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = CGAL::Three::Scene_item::qt_metacall(_c, _id, _a);
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
    return _id;
}
QT_END_MOC_NAMESPACE

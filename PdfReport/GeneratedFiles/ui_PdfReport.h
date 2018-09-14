/********************************************************************************
** Form generated from reading UI file 'PdfReport.ui'
**
** Created by: Qt User Interface Compiler version 5.4.2
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_PDFREPORT_H
#define UI_PDFREPORT_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QDialog>
#include <QtWidgets/QHeaderView>

QT_BEGIN_NAMESPACE

class Ui_PdfReportClass
{
public:

    void setupUi(QDialog *PdfReportClass)
    {
        if (PdfReportClass->objectName().isEmpty())
            PdfReportClass->setObjectName(QStringLiteral("PdfReportClass"));
        PdfReportClass->resize(600, 400);

        retranslateUi(PdfReportClass);

        QMetaObject::connectSlotsByName(PdfReportClass);
    } // setupUi

    void retranslateUi(QDialog *PdfReportClass)
    {
        PdfReportClass->setWindowTitle(QApplication::translate("PdfReportClass", "PdfReport", 0));
    } // retranslateUi

};

namespace Ui {
    class PdfReportClass: public Ui_PdfReportClass {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_PDFREPORT_H

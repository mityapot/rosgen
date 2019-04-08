#!/usr/bin/python3
# -*- coding: utf-8 -*-

import sys
from PyQt5.QtWidgets import (QWidget, QGridLayout, QApplication, QTableWidget, QTableWidgetItem)
from PyQt5.QtCore import Qt



class PSWizard(QWidget):

    def __init__(self):
        super().__init__()

        self.initUI()


    def initUI(self):
        self.setGeometry(900, 300, 700, 400)
        self.setWindowTitle('Менеджер подписчиков и издателей')
        grid_layout = QGridLayout()
        table = QTableWidget(self)  # Создаём таблицу
        table.setColumnCount(3)  # Устанавливаем три колонки
        table.setRowCount(1)  # и одну строку в таблице

        # Устанавливаем заголовки таблицы
        table.setHorizontalHeaderLabels(["Header 1", "Header 2", "Header 3"])

        # Устанавливаем всплывающие подсказки на заголовки
        table.horizontalHeaderItem(0).setToolTip("Column 1 ")
        table.horizontalHeaderItem(1).setToolTip("Column 2 ")
        table.horizontalHeaderItem(2).setToolTip("Column 3 ")

        # Устанавливаем выравнивание на заголовки
        table.horizontalHeaderItem(0).setTextAlignment(Qt.AlignLeft)
        table.horizontalHeaderItem(1).setTextAlignment(Qt.AlignHCenter)
        table.horizontalHeaderItem(2).setTextAlignment(Qt.AlignRight)

        # заполняем первую строку
        table.setItem(0, 0, QTableWidgetItem("Text in column 1"))
        table.setItem(0, 1, QTableWidgetItem("Text in column 2"))
        table.setItem(0, 2, QTableWidgetItem("Text in column 3"))

        # делаем ресайз колонок по содержимому
        table.resizeColumnsToContents()

        grid_layout.addWidget(table, 0, 0, 3,50)  # Добавляем таблицу в сетку
        self.show()





if __name__ == '__main__':
    app = QApplication(sys.argv)
    ex = PSWizard()
    sys.exit(app.exec_())
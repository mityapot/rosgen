#!/usr/bin/python3
# -*- coding: utf-8 -*-

import sys
from PyQt5.QtWidgets import (QWidget, QApplication, QTableWidget, QPushButton, QVBoxLayout, QHBoxLayout, QLineEdit, QComboBox, QMainWindow, QMessageBox )
from PyQt5.QtCore import Qt, pyqtSlot, pyqtSignal
from PyQt5.QtGui import QIntValidator


class ManagerGui(QMainWindow):
    """
    ManagerGui class, window of manager of subscribers and publishers
    """

    def __init__(self, sub_list=list(), pub_list=list()):
        """
        ManagerGui object constructor
        :param sub_list: list of subscribers parameters
        :type sub_list: list
        :param pub_list: list of publishers parameters
        :type pub_list: list
        """

        super().__init__()
        self.sub_list = sub_list
        self.pub_list = pub_list
        self.initUI()

    def initUI(self):
        """
        Function initialisation of manager of subscribers and publishers
        """

        self.wid = PSWizard(self.sub_list, self.pub_list)
        self.setCentralWidget(self.wid)
        self.statusbar = self.statusBar()
        self.statusbar.showMessage('Ожидание данных')
        self.wid.msg2Statusbar[str].connect(self.statusbar.showMessage)
        self.setGeometry(900, 300, 900, 400)
        self.setWindowTitle('Менеджер подписчиков и издателей')
        self.setWindowModality(Qt.ApplicationModal)

    def manager_close(self):
        """
        Function close manager window

        """

        if self.wid.check_last_row() and self.wid.changed is True:
            but = QMessageBox().question(self, 'Message', "Вы точно хотите выйти и не применять изменения?",
                                             QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
            if but == QMessageBox.Yes:
                self.hide()
                self.statusbar.showMessage('Ожидание данных')
                self.wid.reload_table()
        elif self.wid.check_last_row() is False and self.wid.changed is True:
            pass
        else:
            self.hide()

    def keyPressEvent(self, e):
        """
        Function catch hot key press events
        :param e: key pressed event
        :type e: event object
        """

        if e.key() == Qt.Key_Escape:
            self.manager_close()
        if e.key() == Qt.Key_Enter:
            print('Enter')

    def closeEvent(self, QCloseEvent):
        """
        Function catch close manager event
        :param QCloseEvent: close event
        :type QCloseEvent: QCloseEvent
        """

        QCloseEvent.ignore()
        self.manager_close()



class PSWizard(QWidget):
    """
    PSWizard class, widget provides view and edit all subscribers and publishers parameters
    """

    msg2Statusbar = pyqtSignal(str)
    changed = False

    def __init__(self, sub_list=list(), pub_list=list()):
        """
        PSWizard object constructor
        :param sub_list: list of subscribers parameters
        :type sub_list: list
        :param pub_list: list of publishers parameters
        :type pub_list: list
        """

        super().__init__()
        self.table = None
        self.sub_list = sub_list
        self.pub_list = pub_list
        self.initUI()

    def initUI(self):
        """
        Function initialisation of widget interface
        """

        self.setGeometry(900, 300, 900, 400)
        self.setWindowTitle('Менеджер подписчиков и издателей')
        lay = QVBoxLayout(self)
        pushrowButton = QPushButton("Добавить строку")
        pushrowButton.clicked.connect(self.add_row_button)
        make_paramButton = QPushButton("ОК")
        make_paramButton.clicked.connect(self.make_param)

        self.table = QTableWidget()  # Создаём таблицу
        # signal to function change
        self.table.setColumnCount(6)  # Устанавливаем три колонки
        # Устанавливаем заголовки таблицы
        self.table.setHorizontalHeaderLabels(["Подписчик/Издатель", "Имя", "Тип сообщения", "Топик", "Размер очереди", "Удалить"])

        # Устанавливаем выравнивание на заголовки
        self.table.horizontalHeaderItem(0).setTextAlignment(Qt.AlignCenter)
        self.table.horizontalHeaderItem(1).setTextAlignment(Qt.AlignCenter)
        self.table.horizontalHeaderItem(2).setTextAlignment(Qt.AlignCenter)
        self.table.horizontalHeaderItem(3).setTextAlignment(Qt.AlignCenter)
        self.table.horizontalHeaderItem(4).setTextAlignment(Qt.AlignCenter)
        self.table.horizontalHeaderItem(5).setTextAlignment(Qt.AlignCenter)

        # делаем ресайз колонок по содержимому
        self.table.resizeColumnsToContents()
        hbox = QHBoxLayout()
        hbox.addStretch(1)
        hbox.addWidget(pushrowButton)
        hbox.addWidget(make_paramButton)
        lay.addWidget(self.table)
        lay.addLayout(hbox)
        self.setLayout(lay)
        self.setMinimumSize(900, 400)
        self.show()

    @pyqtSlot()
    def deleteClicked(self):
        """
        Function delete row in table if button in row was pressed. Defined as slot.
        """

        button = self.sender()
        if button:
            row = self.table.indexAt(button.pos()).row()
            self.table.removeRow(row)
            self.change_data()

    def add_row(self, rowPosition):
        """
        Function insert row in table
        :param rowPosition: position where row insert
        :type rowPosition: int
        :return: list of insert edit lines and combobox of row
        :rtype: list, QComboBox
        """

        p_holder_list = ["odometry_publisher", "nav_msgs::Odometry", "/odom", "10"]
        self.table.insertRow(rowPosition)
        ed_line_row = list()
        for column in range(1, self.table.columnCount() - 1):
            ed_line = QLineEdit()
            ed_line.textEdited.connect(self.change_data)
            ed_line.setAlignment(Qt.AlignCenter)
            ed_line.setPlaceholderText(p_holder_list[column - 1])
            if column == 4:
                ed_line.setValidator(QIntValidator(1, 100))
            ed_line_row.append(ed_line)
            self.table.setCellWidget(rowPosition, column, ed_line)
        deleteButton = QPushButton("Удалить")
        deleteButton.clicked.connect(self.deleteClicked)
        self.table.setCellWidget(rowPosition, 5, deleteButton)
        combo = QComboBox()
        combo.addItem("publisher")
        combo.addItem("subscriber")
        combo.currentIndexChanged.connect(self.change_data)
        self.table.setCellWidget(rowPosition, 0, combo)
        self.table.resizeColumnsToContents()
        return ed_line_row, combo

    def add_row_button(self):
        """
        Function insert row in table
        :return: 0 - sucsess  or -1 - error if previous row is not full
        :rtype: int
        """

        rowPosition = self.table.rowCount()
        if rowPosition != 0:
            for line in range(1, self.table.columnCount() - 1):
                if self.table.cellWidget(rowPosition-1, line).text() == '':
                    self.msg2Statusbar.emit('Для начала заполните полностью текущую строку')
                    return -1
                elif not self.validate_row(rowPosition - 1):
                    return -1
        self.add_row(rowPosition)
        self.change_data()
        return 0

    def make_param(self):
        """
        Function make subscribers and publishers list from table
        """

        param_list = ['name', 'msg_type', 'topic_name', 'queue_size']
        if self.check_last_row():
            self.pub_list.clear()
            self.sub_list.clear()
            for row in range(self.table.rowCount()):
                row_dict = dict()
                for line in range(1, self.table.columnCount() - 1):
                    row_dict[param_list[line-1]] = self.table.cellWidget(row, line).text()
                if self.table.cellWidget(row, 0).currentText() == 'publisher':
                    self.pub_list.append(row_dict)
                else:
                    self.sub_list.append(row_dict)
            self.msg2Statusbar.emit('Изменения применены')
            self.changed = False
            print(self.pub_list)
            print(self.sub_list)

    def validate_row(self, row):
        """
        Function validate row data
        :param row: position row which validate
        :type row: int
        """
        if self.table.cellWidget(row, 4).hasAcceptableInput():
            return True
        else:
            self.msg2Statusbar.emit(
                    'Вы ввели неправильное значение размера очереди. Оно должно быть целым положительным числом > 0!')
            return False

    def check_last_row(self):
        """
        Function check data in last table row
        :return: correct data or not
        :rtype: bool
        """

        row = self.table.rowCount() - 1
        print(row)
        if row >= 0:
            for line in range(1, self.table.columnCount() - 1):
                    if self.table.cellWidget(row, line).text() != "":
                        if not self.validate_row(row):
                            return False
                        else:
                            continue
                    else:
                        self.msg2Statusbar.emit('Заполните последнюю строку или удалите и повторите ваше действие')
                        return False
        return True

    def reload_table(self):
        """
        Function reload manager's table from subscribers and publishers list
        """

        param_list = ['name', 'msg_type', 'topic_name', 'queue_size']
        for row in range(self.table.rowCount()):
            self.clear_row(row)
            self.table.removeRow(row)
        for i in range(len(self.pub_list)):
            ed_line_row, _ = self.add_row(i)
            for j in range(len(ed_line_row)):
                ed_line_row[j].setText(self.pub_list[i][param_list[j]])
        rows = i + 1
        for i in range(len(self.sub_list)):
            ed_line_row, combo = self.add_row(rows + i)
            combo.setCurrentIndex(1)
            for j in range(len(ed_line_row)):
                ed_line_row[j].setText(self.sub_list[i][param_list[j]])
        self.table.resizeColumnsToContents()
        self.changed = False

    def clear_row(self, row):
        """
        Function clear row data in table
        :param row: row which clear
        :type row: int
        """

        for line in range(1, self.table.columnCount() - 1):
            self.table.cellWidget(row, line).setText("")

    def change_data(self):
        """
        Function set flag if data changed
        """

        if self.changed is not True:
            self.changed = True
            print('True')


if __name__ == '__main__':
    app = QApplication(sys.argv)
    ex = ManagerGui()
    sys.exit(app.exec_())
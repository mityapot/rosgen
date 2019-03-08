#!/usr/bin/python3
# -*- coding: utf-8 -*-
import sys
import os
from PyQt5.QtWidgets import (QWidget, QMainWindow, QLabel, QLineEdit,
    QTextEdit, QGridLayout, QApplication, QPushButton,  QGroupBox)
from PyQt5.QtCore import pyqtSignal
import xml.etree.cElementTree as ET
from genkernel import package
import datetime

class GenGui(QMainWindow):

    def __init__(self):
        super().__init__()

        self.initUI()

    def initUI(self):

        self.wid = RosGenWidget()
        self.setCentralWidget(self.wid)

        self.statusbar = self.statusBar()
        self.statusbar.showMessage('Ожидание данных')
        self.wid.msg2Statusbar[str].connect(self.statusbar.showMessage)
        self.setGeometry(600, 200, 700, 400)
        self.setWindowTitle('Генератор ROS кода')
        self.show()


class RosGenWidget(QWidget):
    msg2Statusbar = pyqtSignal(str)
    full_ed_lines = []
    def __init__(self):
        super().__init__()

        self.initUI()

    def initUI(self):
        lbl_names = ['Название проекта', 'Версия', 'Директория', 'Описание', 'Автор', 'Почта', 'Зависимости проекта', 'Название ноды', 'Зависимости ноды']
        box_names = ['Подписчик', 'Издатель']
        labels = []
        ed_lines = []
        for name in lbl_names:
            labels.append(QLabel(name))
        for i in range(len(labels)):
            ed_lines.append(QLineEdit())
        self.full_ed_lines.extend(ed_lines)
        grid = QGridLayout()
        grid.setSpacing(5)

        for i in range(1, len(labels) + 1):
            for j in range(0, 2):
                if j == 0:
                 grid.addWidget(labels[i - 1], i, j)
                else:
                    grid.addWidget(ed_lines[i - 1], i, j)
        i = 0
        group_data = []
        for name in box_names:
            wd = self.createGroup(name)
            if i ==0:
                grid.addWidget(wd, len(labels) + 1, 0)
            else:
                grid.addWidget(wd, len(labels) + 1, 1)
            group_data.append(wd)
            i+=1
        pushtextButton = QPushButton("Тестовые данные")
        clearButton = QPushButton("Отчистить")
        genButton = QPushButton("Сгенерировать")
        pushtextButton.clicked.connect(self.buttonClicked)
        genButton.clicked.connect(self.buttonClicked)
        clearButton.clicked.connect(self.buttonClicked)
        grid.addWidget(pushtextButton, len(labels) + 2, 1)
        grid.addWidget(clearButton, len(labels) + 2, 0)
        grid.addWidget(genButton, len(labels) + 3, 0, len(labels) + 3, 0)
        self.setLayout(grid)
        self.show()

    def createGroup(self, box_name):
        labels = []
        ed_lines = []
        line_names = ['Имя ', 'Тип сообщения', 'Топик', 'Размер сообщения']
        groupBox = QGroupBox(box_name)
        grid = QGridLayout()
        for name in line_names:
            labels.append(QLabel(name))
        for i in range(len(labels)):
            ed_lines.append(QLineEdit())
        self.full_ed_lines.extend(ed_lines)
        for i in range(1, len(labels) + 1):
            for j in range(0, 2):
                if j == 0:
                 grid.addWidget(labels[i - 1], i, j)
                else:
                    grid.addWidget(ed_lines[i - 1], i, j)

        groupBox.setLayout(grid)

        return groupBox

    def push_test_data(self):
        i = 1
        param_list = []
        dep_str = ''
        dep_node_str = ''
        xml_file = '../genkernel/test.xml'
        tree = ET.ElementTree(file=xml_file)
        root = tree.getroot()
        for child in root:
            if child.tag == 'name':
                param_list.insert(0, child.text)
            if child.tag == 'version':
                param_list.insert(1, child.text)
            if child.tag == 'description':
                param_list.insert(3, child.text)
            if child.tag == 'maintainer':
                param_list.insert(4, child.text)
                param_list.insert(5, child.attrib['email'])
            if child.tag == 'dir':
                param_list.insert(2, child.text)
            if child.tag == 'depend':
                dep_str = dep_str + child.text + ', '
            if child.tag == 'node':
                node = child
        param_list.insert(6, dep_str)
        for child in node:
            if child.tag == 'name':
                param_list.insert(7, child.text)
            if child.tag == 'depend':
                dep_node_str = dep_node_str + child.text + '/' + child.attrib['type'] + ', '
            if child.tag == 'subscribers':
                for sub in child:
                    for param in sub:
                        if param.tag == 'name':
                            param_list.insert(7 + i, param.text)
                        if param.tag == 'msg_type':
                            param_list.insert(8 + i, param.text)
                        if param.tag == 'topic_name':
                            param_list.insert(9 + i, param.text)
                        if param.tag == 'queue_size':
                            param_list.insert(10 + i, param.text)
                    i += 4
            if child.tag == 'publishers':
                for pub in child:
                    for param in pub:
                        if param.tag == 'name':
                            param_list.insert(11 + i, param.text)
                        if param.tag == 'msg_type':
                            param_list.insert(12 + i, param.text)
                        if param.tag == 'topic_name':
                            param_list.insert(13 + i, param.text)
                        if param.tag == 'queue_size':
                            param_list.insert(14 + i, param.text)
                    i += 4
        param_list.insert(8, dep_node_str)
        print(param_list)
        for line, val in zip(self.full_ed_lines, param_list):
            line.setText(val)


    def check_data(self):
        for line in self.full_ed_lines:
            if line.text() != "":
                continue
            else:
                return False
        return True

    def create_gen_xml(self,file):
        param_list = []
        msg = []
        msg_type = []
        for line in self.full_ed_lines:
            param_list.append(line.text())
        dep_pkg = param_list[6].split(',')
        dep_pkg.pop()
        dep_node = param_list[8].split(',')
        dep_node.pop()
        for dep in dep_node:
            a, b = dep.split('/')
            msg.append(a)
            msg_type.append(b)
        print(msg)
        print(msg_type)
        create_dt = datetime.datetime.now()
        fl_name = file + '_' + create_dt.strftime('%d-%m-%Y-%H:%M') +'.xml'
        f = open('../genkernel/templates/my_rosgen.xml')
        o = open('../genkernel/'+ fl_name, 'a')
        flag = 0
        while 1:
            line = f.readline()
            if not line: break
            for i in range(6):
                line = line.replace('[{0}]'.format(i), param_list[i])
            line = line.replace('[7]', param_list[7])
            for i in range(9,17):
                line = line.replace('[{0}]'.format(i), param_list[i])
            if line.find('[6]') != -1:
                for dep in dep_pkg:
                    line_dep = '    <depend>{0}</depend>\n'.format(dep)
                    o.write(line_dep)
                flag = 1
            if line.find('[8]') != -1:
                for dep,tp in zip(msg, msg_type):
                    line_dep = '        <depend type="{1}">{0}</depend>\n'.format(dep, tp)
                    o.write(line_dep)
                flag = 1
            if flag == 0:
                o.write(line)
            else:
                flag = 0
        o.close()
        f.close()
        return  fl_name




    def clear_all_lines(self):
        for line in self.full_ed_lines:
            line.setText("")


    def buttonClicked(self):
        sender = self.sender()
        if sender.text() == 'Тестовые данные':
            self.push_test_data()
            self.msg2Statusbar.emit('Выполнена выгрузка тестовых данных')
        elif sender.text() == 'Сгенерировать':
            if self.check_data():
                file = 'my_rosgen'
                flname = self.create_gen_xml(file)
                os.chdir('../genkernel/')
                pkg = package.RosPackage(flname)
                self.msg2Statusbar.emit('Успешная генерация')
            else:
                self.msg2Statusbar.emit('Не все поля заполнены! Исправьте и повторите генерацию!')
        else:
            self.clear_all_lines()
            self.msg2Statusbar.emit('Произведена полная отчистка полей')



if __name__ == '__main__':

    app = QApplication(sys.argv)
    ex = GenGui()
    sys.exit(app.exec_())
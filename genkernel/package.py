import os
import datetime
from genkernel.node import RosNode
import xml.etree.cElementTree as ET


class RosPackage:
    depend = list()
    pkg_xml = list()
    dir_list = list()

    def __init__(self, xml_file):
        self.create_dt = datetime.datetime.now()
        tree = ET.ElementTree(file=xml_file)
        root = tree.getroot()
        for child in root:
            if child.tag == 'name':
                self.pkg_name = child.text
                self.pkg_xml.append(child)
            if child.tag == 'version':
                self.pkg_xml.append(child)
            if child.tag == 'description':
                self.pkg_xml.append(child)
            if child.tag == 'maintainer':
                self.pkg_xml.append(child)
                author = child.text
            if child.tag == 'dir':
                self.catkin_dir = child.text
            if child.tag == 'depend':
                self.depend.append(child.text)
            if child.tag == 'node':
                node = child
        self.make_dirs()
        self.make_xml()
        self.make_cmake()
        self.pkg_param = {'name': self.pkg_name, 'author': author, 'str_date': self.create_dt.strftime("%B %d, %Y")}
        self.node = RosNode(node, self.pkg_param)
        out_topics = self.node.generate_node_code(self.dir_list[1])
        self.make_readme(out_topics)

    def make_dirs(self):
        self.dir_list = [self.pkg_name, self.pkg_name + '/src', self.pkg_name + '/launch',self.pkg_name + '/cfg', self.pkg_name + '/include']
        self.dir_list = [self.catkin_dir + pkg_dir for pkg_dir in self.dir_list]
        for path in self.dir_list:
            try:
                os.mkdir(path)
            except OSError:
                print("Создать директорию %s не удалось" % path)
            else:
                print("Успешно создана директория %s " % path)

    def make_xml(self):
        tree = ET.ElementTree(file='./templates/package.xml')
        root = tree.getroot()
        build_dep_num = 0
        exec_dep_num = 0
        for i in range(len(root)):
            for child in self.pkg_xml:
                if root[i].tag == child.tag:
                    root[i].text = child.text
                    root[i].attrib = child.attrib
            if root[i].tag == 'build_depend':
                build_dep_num = i + 1
            if root[i].tag == 'exec_depend':
                exec_dep_num = i + 1 + len(self.depend)
        for depend in self.depend:
            b_dep_element = ET.Element('build_depend')
            b_dep_element.text = depend
            root.insert(build_dep_num, b_dep_element)
            build_dep_num += 1
        for depend in self.depend:
            ex_dep_element = ET.Element('exec_depend')
            ex_dep_element.text = depend
            root.insert(exec_dep_num, ex_dep_element)
            exec_dep_num += 1
        tree = ET.ElementTree(root)
        tree.write(self.dir_list[0] + '/package.xml')

    def make_cmake(self):
        f = open('./templates/CMakeLists.txt')
        o = open(self.dir_list[0] + '/CMakeLists.txt', 'a')
        dep_flag = 0
        while 1:
            line = f.readline()
            if not line: break
            line = line.replace('[pkg_name]', self.pkg_name)
            if line.find('roscpp') != -1 and dep_flag != -1:
                dep_flag = 1
            o.write(line)
            if dep_flag == 1:
                for depend in self.depend:
                    o.write("  " + depend + '\n')
                dep_flag = -1
        o.close()
        f.close()

    def make_readme(self, out_topics):
        f = open('./templates/README.md')
        o = open(self.dir_list[0] + '/README.md', 'a')
        dep_flag = 0
        sub_flag = 0
        pub_flag = 0
        while 1:
            line = f.readline()
            if not line: break
            line = line.replace('[pkg_name]', self.pkg_name)
            line = line.replace('[author]', self.pkg_param['author'])
            if line.find('[roscpp](http://wiki.ros.org/roscpp)') != -1 and dep_flag != -1:
                dep_flag = 1
            if line.find('Subscribed topics:') != -1 and sub_flag != -1:
                sub_flag = 1
            if line.find('Published topics:') != -1 and pub_flag != -1:
                pub_flag = 1
            o.write(line)
            if dep_flag == 1:
                for depend in self.depend:
                    o.write('* {}\n'.format(depend))
                dep_flag = -1
            if sub_flag == 1:
                for sub_str in out_topics['subscribed']:
                    o.write('* {}\n'.format(sub_str))
                sub_flag = -1
            if pub_flag == 1:
                for pub_str in out_topics['published']:
                    o.write('* {}\n'.format(pub_str))
                pub_flag = -1
        o.close()
        f.close()



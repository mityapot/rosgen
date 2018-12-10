class RosNode:
    depend = list()
    subscribers = list()
    publishers = list()
    template_names = dict()

    def __init__(self, node_subtree, pkg_param):
        self.pkg_param = pkg_param
        for child in node_subtree:
            if child.tag == 'name':
                self.name = child.text
            if child.tag == 'depend':
                str_depend = child.text + '/' + child.attrib['type']
                self.depend.append(str_depend)
            if child.tag == 'subscribers':
                for sub in child:
                    sub_data = dict()
                    for param in sub:
                        if param.tag == 'name':
                            sub_data['name'] = param.text
                        if param.tag == 'msg_type':
                            sub_data['msg_type'] = param.text
                        if param.tag == 'topic_name':
                            sub_data['topic_name'] = param.text
                        if param.tag == 'queue_size':
                            sub_data['queue_size'] = param.text
                    self.subscribers.append(sub_data)
            if child.tag == 'publishers':
                for pub in child:
                    pub_data = dict()
                    for param in pub:
                        if param.tag == 'name':
                            pub_data['name'] = param.text
                        if param.tag == 'msg_type':
                            pub_data['msg_type'] = param.text
                        if param.tag == 'topic_name':
                            pub_data['topic_name'] = param.text
                        if param.tag == 'queue_size':
                            pub_data['queue_size'] = param.text
                    self.publishers.append(pub_data)

    def generate_node_code(self, dir):
        self.make_node_interface(dir)
        self.make_node_realisation(dir)
        self.make_main(dir)
        sub_str_list = list()
        pub_str_list = list()
        for subscriber in self.subscribers:
            sub_str = '{} - {} topic, `{}`'.format(subscriber['topic_name'], subscriber['name'].split('_')[0], subscriber['msg_type'])
            sub_str_list.append(sub_str)
        for publisher in self.publishers:
            pub_str = '{} - {} topic, `{}`'.format(publisher['topic_name'], publisher['name'].split('_')[0], publisher['msg_type'])
            pub_str_list.append(pub_str)
        out_topics = {'subscribed': sub_str_list, 'published': pub_str_list}
        return out_topics

    def make_node_interface(self, dir):
        name_list = self.name.split('_')
        self.template_names['flname'] = name_list[0] + '_' + name_list[1]
        self.template_names['classname'] = name_list[0].capitalize() + name_list[1].capitalize()
        self.template_names['preproc_dir'] = name_list[0].upper() + '_' + name_list[1].upper() + '_HPP'

        f = open('./templates/node.hpp')
        o = open(dir + '/' + self.template_names['flname'] + '.hpp', 'a')
        dep_flag = 0
        block_flag = 0
        while 1:
            line = f.readline()
            if not line: break
            line = line.replace('[pkg_name]', self.pkg_param['name'])
            line = line.replace('[class_name]', self.template_names['classname'])
            line = line.replace('[date]', self.pkg_param['str_date'])
            line = line.replace('[author]', self.pkg_param['author'])
            line = line.replace('[preproc_dir]', self.template_names['preproc_dir'])
            if line.find('<ros/ros.h>') != -1 and dep_flag != -1:
                dep_flag = 1
            if line.find('ros::NodeHandle _node;') != -1 and block_flag != -1:
                block_flag = 1
            o.write(line)
            if dep_flag == 1:
                for depend in self.depend:
                    o.write('#include <{}.h>\n'.format(depend))
                dep_flag = -1
            if block_flag == 1:
                for subscriber in self.subscribers:
                    o.write('  ros::Subscriber _{};\n'.format(subscriber['name']))
                for publisher in self.publishers:
                    o.write('  ros::Publisher _{};\n'.format(publisher['name']))
                o.write('\n')
                for subscriber in self.subscribers:
                    o.write('  void {}Callback(const {}& message);\n'.format(subscriber['name'].split('_')[0], subscriber['msg_type']))
                block_flag = -1
        o.close()
        f.close()

    def make_node_realisation(self, dir):
        f = open('./templates/node.cpp')
        o = open(dir + '/' + self.template_names['flname'] + '.cpp', 'a')
        sub_flag = 0
        pub_flag = 0
        while 1:
            line = f.readline()
            if not line: break
            line = line.replace('[pkg_name]', self.pkg_param['name'])
            line = line.replace('[class_name]', self.template_names['classname'])
            line = line.replace('[date]', self.pkg_param['str_date'])
            line = line.replace('[author]', self.pkg_param['author'])
            line = line.replace('[flname]', self.template_names['flname'])
            if line.find('// subscribers settings') != -1 and sub_flag != -1:
                sub_flag = 1
            if line.find('// publishers settings') != -1 and pub_flag != -1:
                pub_flag = 1
            o.write(line)
            if sub_flag == 1:
                for subscriber in self.subscribers:
                    o.write('  _{} = _node.subscribe(\'{}\', {}, &{}::{}Callback, this);\n'.format(subscriber['name'], subscriber['topic_name'], subscriber['queue_size'], self.template_names['classname'], subscriber['name'].split('_')[0]))
                sub_flag = -1
            if pub_flag == 1:
                for publisher in self.publishers:
                    o.write('  _{} = _node.advertise<{}>(\'{}\', {});\n'.format(publisher['name'], publisher['msg_type'], publisher['topic_name'], publisher['queue_size']))
                pub_flag = -1
        f.close()
        o.write('\n')
        for subscriber in self.subscribers:
            o.write('void {}::{}Callback(const {}& m) '.format(self.template_names['classname'], subscriber['name'].split('_')[0], subscriber['msg_type']))
            o.write('{\n}\n')
        o.close()

    def make_main(self, dir):
        f = open('./templates/node_main.cpp')
        o = open(dir + '/' + self.template_names['flname'] + '_main.cpp', 'a')
        while 1:
            line = f.readline()
            if not line: break
            line = line.replace('[pkg_name]', self.pkg_param['name'])
            line = line.replace('[class_name]', self.template_names['classname'])
            line = line.replace('[date]', self.pkg_param['str_date'])
            line = line.replace('[author]', self.pkg_param['author'])
            line = line.replace('[flname]', self.template_names['flname'])
            line = line.replace('[node_name]', self.name)
            o.write(line)
        o.close()
        f.close()
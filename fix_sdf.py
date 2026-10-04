import xml.etree.ElementTree as ET

tree = ET.parse('src/custom_corridor/worlds/school_arena/school_arena_hunav.sdf')
root = tree.getroot()
world = root.find('world')

for model in world.findall('model'):
    name = model.get('name', '')
    if name.startswith('student_'):
        link = model.find('base_link')
        if link is not None and link.find('collision') is None:
            col = ET.fromstring('''<collision name="collision">
          <pose>0 0 0.675 0 0 0</pose>
          <geometry><cylinder><radius>0.22</radius><length>1.35</length></cylinder></geometry>
          <surface><contact><collide_bitmask>0x01</collide_bitmask></contact></surface>
        </collision>''')
            # Insert before visual
            idx = 0
            for i, child in enumerate(link):
                if child.tag == 'visual':
                    idx = i
                    break
            link.insert(idx, col)

tree.write('src/custom_corridor/worlds/school_arena/school_arena_hunav.sdf', encoding='UTF-8', xml_declaration=True)

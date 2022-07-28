import stuff
stuff.clear()
import mplcursors
from matplotlib import pyplot

# Rage generated ** per second ** from auto attacks (Fury, regardless of weapon speeds)
#   Main hand: 1.75
#   Off hand: 0.875
#   Average: 1.3125

# Rage generated ** per swing ** (Fury, regardless of weapon speeds)
#   Main hand: 6.3 two-handed, 4.55 one-handed
#   Off hand: 3.15 two-handed, 2.275 one-handed
#   Average: 4.725 two-handed, 3.4125 one-handed

HASTE_RATING_TO_PERCENT = 15.8      # 15.8 haste rating = 1% haste

BOSS_ARMOR = {
    'Hydross the Unstable': 7700,
    'The Lurker Below': 7700,
    'Leotheras the Blind': 7700,
    'Fathom-Lord Karathress': 6200,
    'Morogrim Tidewalker': 7700,
    'Lady Vashj': 6200,
    'Void Reaver': 8800,
    'High Astromancer Solarian': 6200,
    'Al\'ar': 7700,
    'Kael\'thas Sunstrider': 6200,
    'Rage Winterchill': 6200,
    'Anetheron': 6200,
    'Kaz\'rogal': 6200,
    'Azgalor': 6200,
    'Archimonde': 6200,
    'High Warlord Naj\'entus': 7700,
    'Supremus': 7700,
    'Shade of Akama': 7700,
    'Teron Gorefiend': 6200,
    'Gurtogg Bloodboil': 7700,
    'Essence of Suffering': 0,
    'Essence of Desire': 7700,
    'Essence of Anger': 7700,
    'Mother Shahraz': 6200,
    'Gathios the Shatterer': 6200,
    'Illidan Stormrage': 7700
}

# max armor reduction = 4,010
def damage_reduction(boss_name, armor=None, max_armor_reduction=True, mob_level=73):
    if armor is None:
        if boss_name in BOSS_ARMOR:
            if max_armor_reduction:
                armor = BOSS_ARMOR[boss_name] - 4010
            else: armor = BOSS_ARMOR[boss_name]
        else: return
    else:
        if max_armor_reduction:
            armor = armor - 4010

    denominator = armor - 22167.5 + (467.5 * mob_level)
    dr = armor / denominator
    return round(dr*100,2)

# print(str(damage_reduction('Illidan Stormrage'))+'%\n')

armor_penentration = [i for i in range(3691)]
boss_damage_reduction = [damage_reduction('Illidan Stormrage',armor=7700-i) for i in range(3691)]

pyplot.style.use('ggplot')
pyplot.plot(armor_penentration,boss_damage_reduction)
pyplot.xlabel('Armor Penetration', fontweight='bold')
pyplot.ylabel('Boss\'s Damage Reduction  (%)', fontweight='bold')
pyplot.title('Boss DR%  vs.  Armor Penetration', fontweight='bold')
mplcursors.cursor(hover=True)
pyplot.show()


# curve = [(-0.000000461713*x**2 - 0.004639114484*x + 23.504378199823) for x in armor_penentration]

# def derivative(curve):
#     return [curve[i+1]-curve[i] for i in range(len(curve)-1)]

# x = [i for i in range(3690)]
# d_curve = derivative(curve)
# pyplot.plot(x,d_curve)
# pyplot.show()
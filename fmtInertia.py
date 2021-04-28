import re
import pyperclip

inertiaString = input("Enter String")
for i in range(33):
    inertiaString = inertiaString + "\n"+ input()


mass = re.search("Mass = (\d+\.\d+) kilograms", inertiaString).group(1)
inertia = "".join(inertiaString.split("\n")[-3:]).replace(
    "I", "i").replace(" = ", "=").replace("\t", " ").replace("\n", " ")


inertia = re.sub("(\d+\.\d+)", r'"\1"', inertia)



urdfString = f"<mass value=\"{mass}\"/>\n<inertia {inertia}/>"

pyperclip.copy(urdfString)

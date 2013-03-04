
Welcome to Lasercake!
=====================

Lasercake is an open-world game about the environment.

Website: [lasercake.net](http://www.lasercake.net/)

We appreciate feedback.
Any way is good to contact us: [email](mailto:lasercake@googlegroups.com), [Google Group](https://groups.google.com/d/forum/lasercake), [IRC](https://webchat.freenode.net/?channels=lasercake) or [GitHub](https://github.com/Lasercake/Lasercake/issues).


Project blurb
=============

What is energy? Where does the energy we use come from? How do we gather and process materials, and what effects does that have on the planet we live on?

The Lasercake project aims to help people understand these things through the powerful medium of computer games.

**In Lasercake (the game), the player can use robots to build industrial projects – but unlike in similar games, every part of the world is based on real-life science.** Mine waste has to be dumped somewhere and causes pollution. Energy is conserved. Solar panels, wind turbines, and so on, harvest realistic amounts of energy. In short, we plan to include any and all scientific concepts that we can include while still keeping the game fun and engaging.

Most of those things don't exist yet – the game is far from complete – but we have a prototype showing some of the things we've done already.

We want to make awesome things free to everyone. Anyone with an Internet connection may download Lasercake and its source code without charge, and anyone with the ability may create and distribute modified versions under the terms of the GNU AGPL.

The current project team is [Eli Dupree](http://www.elidupree.com/) and [Isaac Dupree](http://www.idupree.com/). We began this project in December 2011 as an experiment in simulating water physics, and it's only kept expanding since.

We want your cool skills! The two of us could do this project on our own, but **it will be more awesome if you help us out.** There's a lot of different things that go into a big project like Lasercake – art, sustainable design, computer programming, geology, physics, sound design,  and many other things besides. If you want to help, any way is good to contact us: [email](mailto:lasercake@googlegroups.com), [Google Group](https://groups.google.com/d/forum/lasercake), [IRC](https://webchat.freenode.net/?channels=lasercake) or [GitHub](https://github.com/Lasercake/Lasercake/issues).


Game instructions
=================

When you start the game, click in the game window – then you'll be able to look around in-game using the mouse. You'll be looking from the point of view of a robot who can dig, shoot lasers, and build stuff. **All of the controls are written in-game** – you can probably learn most of the game by just trying all the controls that are listed, and reading the text that pops up. In fact, why don't you do that right now? You can come back and read the rest of this README if you get stuck.

The main controls are:

* Mouse motion: Look around.
* WASD: Move in the four horizontal directions, relative to the way you're looking.
* Space: Move upwards.
* (left) mouse button: Do whichever action you have selected.
* ZXCVB: Select different actions (they show descriptions when selected).

Your robot also levitates a little off the ground, so you can climb up shallow inclines with just the WASD keys.

You start out looking at a "refinery". Right now, it's a white box, but eventually we will make it look cooler. The resource flow currently goes like this:

You need metal to build things. You start with some metal, but to get more, you need to refine it. The simplest way to do that is...

1. Be in digging mode. (You start in digging mode, and you can switch back with "z").
2. Point at a nearby rock tile and click it turn turn it to rubble, then click it again to shove it.
3. Keep shoving the rock until it gets to the refinery input.
4. Repeat until you've gathered enough to make one tile of metal. (The refinery splits rubble into metal and waste rock – each rubble tile only has a little metal in it. The orangeish waste rock comes out of the refinery opposite the input, and the yellow metal comes out the side. Greener rock has more metal in it.)
5. Go near the yellow metal tile and click it to pick it up (note that you can't do this if you're still full of metal – each tile is 200 cubic meters and you can only carry a total of 421 cubic meters).

That's a pretty slow process. There's a lot of ways to do it faster. You can use lasers ("x") to disintegrate more rock at once. You can build more conveyors ("c") to move rubble automatically. You can build other robots ("b") to automatically dig for you. And you can build extra refineries ("v"), although there currently isn't any real advantage to using more than one.

You can also deconstruct buildings and retreive their metal ("z" mode and click a nearby building), as long as you have enough room to store the extra metal.

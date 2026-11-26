Ukázkové programy
=================

Tyto příklady můžete otevřít přímo v Normálním režimu přes nabídku **Příklady & Cvičení**,
spustit je a libovolně upravovat. Každý skript si vytváří vlastního klienta a vybírá svět.

Kloubové řízení
---------------

Nastavení cílových úhlů jednotlivých kloubů tak, aby robot rukou posunul míček.

.. literalinclude:: ../../icub_pybullet/examples/push_the_ball_pure_joints.py
   :pyobject: push_the_ball
   :language: python

Kartézské řízení
----------------

Posun koncového efektoru (ruky) v prostoru vzhledem k jeho aktuální pozici.

.. literalinclude:: ../../icub_pybullet/examples/push_the_ball_cartesian.py
   :pyobject: push_the_ball
   :language: python

Dotyková kůže
-------------

Svět `skin_test.yaml` ukazuje, které senzory na umělé kůži se aktivují při dopadu míčků na robota.

.. literalinclude:: ../../icub_pybullet/examples/skin_test.py
   :pyobject: main
   :language: python

Minimální smyčka simulace
-------------------------

:download:`Stáhněte si startovací skript <empty_example.py>` a otevřete jej v Normálním režimu.
Vlastní příkazy pro robota přidejte za vytvoření klienta.

.. literalinclude:: empty_example.py
   :language: python

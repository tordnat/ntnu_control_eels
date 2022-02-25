# Jumping

The snake:
```
-*-*-*-*-*-
```

is lifting one arm on each side
```
\         /
 *-*-*-*-*
```

or multiple arms
```
\     /
 \   /
  \ /
   *
```

to a target angle of
$$ \theta = \ ? $$

Mass and length of each arm piece:
$$ m = 8.6\mathrm{kg} $$
$$ l = 0.4\mathrm{m} $$

Total mass of vehicle:
$$ M = 80\mathrm{kg} $$

The moment of inertia of an arm consisting of $n$ pieces is:
$$ I = \frac{1}{3}(nm)(nl)^2 = \frac{1}{3} m l^2 n^3 $$

The torque of each joint is
$$ \tau = 5 \mathrm{Nm} $$

Which results in an angular acceleration 
$$ \alpha = \frac{\tau}{I} $$

With constant angular acceleration, we can solve for the time required to reach the angle $\theta$
$$ \theta = \frac{1}{2} \alpha t^2 $$
$$ t = \sqrt{\frac{2\theta}{\alpha}} $$

Which leaves us with the angular velocity
$$ \omega = \alpha t $$

Which finally can be converted to rotational energy
$$ K_r = I \omega^2 $$
(times two since we have two arms rotating)

The arms hit their mechanical limits at $\theta$ degrees, thus stopping abruptly.
Lets assume that the amount of energy
transferred into "upwards" motion
is proportional to $\cos(\theta)$.
Then we get the upwards kinetic energy:
$$ K_e = I \omega^2 \cos(\theta) $$

Then it's trivial to calculate our maximum height:
$$ h = \frac{I \omega^2 \cos(\theta)}{Mg} $$

Expressing in terms of $\theta$:
$$
h = \frac{I (\alpha t)^2 \cos (\theta)}{Mg}
$$
$$
 = \frac{2 I \theta \alpha  \cos (\theta)}{Mg}
$$
$$
 = \frac{2 \theta \tau \cos (\theta)}{Mg}
$$

Immediately after liftoff, we would have the velocity
$$ v = \sqrt{\frac{2K_e}{M}} $$ 
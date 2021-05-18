#! /usr/bin/env python3
# -*- coding:utf-8 -*-
from relampago_markinhos import RelampagoMarkinhos
from creeper import Creeper
from prints import logo_relampago_markinhos

from config import config as c

if __name__ == "__main__":
    # logo_relampago_markinhos(c.OBJETIVO, c.PRINTS_INICIAIS)
    creeper_objetivo = Creeper(c.OBJETIVO[0],c.OBJETIVO[1])
    RelampagoMarkinhos(c.OBJETIVO, creeper_objetivo, c.CONCEITO_C)



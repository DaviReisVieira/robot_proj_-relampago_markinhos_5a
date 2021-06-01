#! /usr/bin/env python3
# -*- coding:utf-8 -*-
from relampago_markinhos import RelampagoMarkinhos
from creeper import Creeper
from estacao import Estacao
from prints import logo_relampago_markinhos

from config import config as c

if __name__ == "__main__":
    OBJETIVO, CONCEITO_C, SOMENTE_PISTA, ESQUERDA, PRINTS_INICIAIS = c.configuracoes_markinhos()
    logo_relampago_markinhos(OBJETIVO, PRINTS_INICIAIS)
    creeper_objetivo = Creeper(OBJETIVO[0],OBJETIVO[1])
    estacao_objetivo = Estacao(OBJETIVO[2])
    RelampagoMarkinhos(OBJETIVO, creeper_objetivo, estacao_objetivo, CONCEITO_C, SOMENTE_PISTA, ESQUERDA)



##========================================================================##
#                       __ _                            /\/|               #
#                      / _(_)                          |/\/                #
#       ___ ___  _ __ | |_ _  __ _ _   _ _ __ __ _  ___ ___   ___  ___     #
#      / __/ _ \| '_ \|  _| |/ _` | | | | '__/ _` |/ __/ _ \ / _ \/ __|    #
#     | (_| (_) | | | | | | | (_| | |_| | | | (_| | (_| (_) |  __/\__ \    #
#      \___\___/|_| |_|_| |_|\__, |\__,_|_|  \__,_|\___\___/ \___||___/    #
#                             __/ |                 )_)                    #
#                            |___/                                         #
##========================================================================##
from prints import configs

##================================ MISSÕES ===============================##
missao_1 = ("blue", 12, "dog")
missao_2 = ("green", 23, "horse")
missao_3 = ("orange", 11, "cow")
missoes = {'1':missao_1,'2':missao_2,'3':missao_3}
##============================= CONFIGURAÇÕES=============================##

def configuracoes_markinhos(tests=False):
    # Coloque na Variável 'OBJETIVO' qual Missão você deseja realizar!
    OBJETIVO = missao_1

    # Caso queira testar o Conceito C, mude para True a variável 'CONCEITO_C'
    CONCEITO_C = False

    # Caso queira testar o Conceito C (Apenas rodar a pista toda), mude para True a variável 'CONCEITO_C'
    SOMENTE_PISTA = False

    # Para desligar as prints iniciais, mude 'PRINTS_INICIAIS' para False
    PRINTS_INICIAIS = True

    if not tests:
        configs()

        print("""
Missão 1 = ("blue", 12, "dog")
Missão 2 = ("green", 23, "horse")
Missão 3 = ("orange", 11, "cow")
        """)
        objetivo_creeper = input('Escolha uma Missão: (1, 2, 3, custom / c): ')
        if objetivo_creeper == 'custom' or objetivo_creeper == 'c':
            cor = input('Escolha uma cor: (blue, green, orange): ')
            id_creeper = int(input('Escolha o ID: (11, 12, 13, 21, 22, 23): '))
            estacao = input('Escolha uma estação: (dog, horse, cow): ')
            OBJETIVO = (cor,id_creeper,estacao)
        else:
            OBJETIVO = missoes[objetivo_creeper]

        conceito_c = input('Deseja testar o Conceito C? (Y/N): ')

        if conceito_c == 'Y' or conceito_c == 'y':
            CONCEITO_C = True
            somente_segue_pista = input('Deseja testar apenas o Seguidor de Linha do Conceito C? (Y/N): ')
            if somente_segue_pista == 'Y' or somente_segue_pista == 'y':
                SOMENTE_PISTA = True

    return OBJETIVO, CONCEITO_C, SOMENTE_PISTA, PRINTS_INICIAIS
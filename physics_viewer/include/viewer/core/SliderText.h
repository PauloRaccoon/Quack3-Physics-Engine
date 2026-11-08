// =============================================================================
// SliderText.h  (V20)
// Parte sem raylib da edição de sliders por texto (clique duplo) e da quebra de linhas dos painéis:
// ler o número digitado, converter entre o valor interno do slider e o que aparece na tela, e quebrar
// um texto longo (caminho de arquivo) em linhas que cabem numa largura.
// =============================================================================
#pragma once

#include <functional>
#include <string>
#include <vector>

namespace viewer {

/// Como o valor do slider aparece para o usuário (e portanto como o texto digitado é lido):
/// exibido = (square ? v·v : v) · scale. Ex.: massa (v = raiz da massa) {1, true}; slop em mm {1000, false}.
struct SliderUnit {
    float scale = 1.0f;
    bool square = false;
};

/// V24: calcula uma expressão aritmética: + - * / , parênteses, sinal unário, vírgula ou ponto decimal, notação
/// científica ("1e3") e, no fim, um sufixo de unidade só com letras/símbolos ("2,5 m"). "4/2" -> 2, "1-4" -> -3,
/// "2*(3+1)" -> 8. false (e `out` intacto) se a sintaxe é inválida, divide por zero, o resultado não é finito
/// ou o aninhamento passa de 64 níveis.
bool evalExpression(const std::string& text, double& out);

/// Lê um número do texto (agora com contas: ver evalExpression): "2,5 m" -> 2.5, "4/2" -> 2.
/// false se não há número ou ele não é finito ("abc", "", "inf", "nan").
bool parseNumber(const std::string& text, double& out);

/// Texto digitado -> valor interno do slider, limitado a [lo, hi]. false (e `out` intacto) se o texto não é um número.
/// unit.square: o número digitado é o QUADRADO do valor interno (negativo vira 0). scale 0 é tratado como 1.
bool sliderValueFromText(const std::string& text, const SliderUnit& unit, float lo, float hi, float& out);

/// Valor interno -> texto inicial do campo de edição (até 5 algarismos significativos, sem zeros à toa).
std::string sliderTextFromValue(float v, const SliderUnit& unit);

/// Quebra `text` em até `maxLines` linhas de largura <= `maxWidth` segundo `measure` (largura em px de uma string).
/// Prefere quebrar depois de '/', '\\', '_' ou espaço no último terço da linha; se ainda sobrar texto na última
/// linha, ela termina em "..". Nunca devolve linha vazia; largura <= 0 ou texto vazio: nenhuma linha.
std::vector<std::string> wrapText(const std::string& text, int maxWidth, int maxLines, const std::function<int(const std::string&)>& measure);

} // namespace viewer

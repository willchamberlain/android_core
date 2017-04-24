package william.chamberlain.androidvosopencvros;

import java.util.regex.Matcher;

import static william.chamberlain.androidvosopencvros.DataExchange.dataOutputPattern;


/**
 * Created by will on 24/04/17.
 */

public class DataOutputPatternTest {
//    public static final Pattern dataOutputPattern = Pattern.compile("data ( [A-z_0-9]:[A-z_0-9'\"\\.])+ ");


    public static void main(String[] args){
        String putativeMatch;
        assertPatternShouldNotMatch("data");
        assertPatternShouldNotMatch("data ");
        assertPatternShouldNotMatch("data thisValue");
        assertPatternShouldNotMatch("data thisValue:");
        assertPatternShouldMatch("data thisValue:5.05", 2, "thisValue", "5.05");
        assertPatternShouldMatch("data thatValue:I_like_regex_patterns", 2, "thatValue", "I_like_regex_patterns");
    }

    private static void assertPatternShouldNotMatch(String shouldntMatch_1) {
        String putativeMatch;
        putativeMatch = shouldntMatch_1;
        Matcher matcher = dataOutputPattern.matcher(putativeMatch);
        if(matcher.matches()) {
            throw new RuntimeException("ERROR: pattern '"+dataOutputPattern.toString()+"' should not match string '"+putativeMatch+"'");
        } else {
            System.out.println("Correct: pattern '"+dataOutputPattern.toString()+"' did not match string '"+putativeMatch+"'");
        }
    }

    private static void assertPatternShouldMatch(String shouldMatch_1, int numberOfMatches, String groupOne, String groupTwo) {
        String putativeMatch;
        putativeMatch = shouldMatch_1;
        Matcher matcher = dataOutputPattern.matcher(putativeMatch);
        if(matcher.matches()) {
            System.out.println("Correct: pattern '"+dataOutputPattern.toString()+"' does match string '"+putativeMatch+"'");
        } else {
            throw new RuntimeException("ERROR: pattern '"+dataOutputPattern.toString()+"' should match string '"+putativeMatch+"'");
        }
        if( numberOfMatches == matcher.groupCount() ) { // "Group zero denotes the entire pattern by convention. It is not included in this count. " groups 1..n are the matched groups
            System.out.println("Correct: pattern '"+dataOutputPattern.toString()+"' does match "+numberOfMatches+" groups in string '"+putativeMatch+"'");
        } else {
            System.out.println("ERROR: pattern '" + dataOutputPattern.toString() + "' should match " + numberOfMatches + " NOT " + matcher.groupCount() + " groups in string '" + putativeMatch + "'");
        }
        if( groupOne.equals(matcher.group(1)) ) { // "Group zero denotes the entire pattern by convention. It is not included in this count. " groups 1..n are the matched groups
            System.out.println("Correct: group 1: '"+groupOne+"' does match '"+matcher.group(1)+"'  in string '"+putativeMatch+"'");
        } else {
            System.out.println("ERROR: group 1: '"+groupOne+"' does NOT match '"+matcher.group(1)+"'  in string '"+putativeMatch+"'");
        }
        if( groupTwo.equals(matcher.group(2)) ) { // "Group zero denotes the entire pattern by convention. It is not included in this count. " groups 1..n are the matched groups
            System.out.println("Correct: group 2: '"+groupTwo+"' does match '"+matcher.group(2)+"'  in string '"+putativeMatch+"'");
        } else {
            System.out.println("ERROR: group 2: '"+groupTwo+"' does NOT match '"+matcher.group(2)+"'  in string '"+putativeMatch+"'");
        }
    }
}
